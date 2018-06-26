# pms5003_micropython
Driver for pms5003 air quality sensor for micropython

## Description and features
This driver for the [pms5003 air quality sensor](http://www.aqmd.gov/docs/default-source/aq-spec/resources-page/plantower-pms5003-manual_v2-3.pdf) is specifically made for micropython.
It can be used with esp32 (tested on [esp32_loboris fork](https://github.com/loboris/MicroPython_ESP32_psRAM_LoBo)) and will not work on esp8266 as it only has one UART.
It is completely based on uasyncio

The driver covers all features of the device:
* sleep mode and wake up by pin and uart
* reset
* passive mode
* active mode
* changing modes during runtime

The library is built to be reslient, so if a reset pin is given, it will reset the device after a few tries if commands are not answered or no new data is received within 60s in active mode. 
If a reset fails (even because no reset pin is given) it will stop the device.

Reset and Set pin are completely optional though. The reset pin could make sense to have, the Set pin for sleep/wakeUp seems unnecessary as it's functionality is covered by uart commands.

## Dependencies

* uasyncio (Version >= 2.0)

## How to use
```
import pms5003
import machine
import uasyncio as asyncio

class Lock:
    def __init__(self):
        self._locked = False

    async def __aenter__(self):
        while True:
            if self._locked:
                await asyncio.sleep_ms(_DEFAULT_MS)
            else:
                self._locked = True
                break

    async def __aexit__(self, *args):
        self._locked = False
        await asyncio.sleep_ms(_DEFAULT_MS)

    def locked(self):
        return self._locked

    def release(self):
        self._locked = False
        
lock = Lock()
uart = machine.UART(1, tx=25, rx=26, baudrate=9600)
pm = pms5003.PMS5003(uart, lock) 
pm.registerCallback(pm.print)

loop=asyncio.get_event_loop()
loop.run_forever()
```

An output of pm.print() looks like this:
```
---------------------------------------------
Measurement 12ms ago at 2018-06-24 22:28:21
---------------------------------------------
Concentration Units (standard)
---------------------------------------------
PM 1.0: 1       PM2.5: 2        PM10: 2
Concentration Units (environmental)
---------------------------------------------
PM 1.0: 1       PM2.5: 2        PM10: 2
---------------------------------------------
Particles > 0.3um / 0.1L air: 528
Particles > 0.5um / 0.1L air: 151
Particles > 1.0um / 0.1L air: 18
Particles > 2.5um / 0.1L air: 0
Particles > 5.0um / 0.1L air: 0
Particles > 10 um / 0.1L air: 0
---------------------------------------------
```
 
After the creation of the pms5003 object, which is configurable in the constructor, the library takes care of everything and you just have to register your callbacks or events.
Of course you can change everything later as you see fit and change the reading mode, the eco mode or the polling interval in passive mode.
 
## Methods explained
* __init__() has many options:
  * uart:       a uart object is required
  * lock:       an asyncio lock object is needed too. If you don't have one, copy it from the example
  * set_pin:    optional, a pin for putting to sleep/waking up, functionality covered by uart
  * reset_pin:  optional, a pin for resetting the device in case of an error
  * interval_passive_mode: optional, defaults to 60s, the device will be read in this interval in passive mode
  * event:      optional, you can define an asyncio event that will be set if new data is available
  * active_mode:  boolean, defaults to True; in active_mode the device will send new data every few seconds and the library just listens; in passive mode interval_passive_mode is used to periodically request data from the device
  * eco_mode:   boolean, defaults to True, if the device is in passive mode, then it will be put to sleep between data requests
  * assume_sleeping: boolean, defaults to True; assume that the device is in sleeping state when library starts. If set to False and device happens to be in sleeping state (e.g. because of microcontroller reset) the library will probably fail as it can't read anything. Only change this if you know why you do it.
* _error/_warn/_debug: These are logging functions and can be adapted in a subclass
* setEcoMode(boolean): Change EcoMode
* async setActiveMode():  Set mode to active
* async setPassiveMode(interval=None): Set mode to passive, interval optional, defaults to 60s or the interval given in the constructor
* async sleep():  Put device to sleep
* async wakeUp(): Wake device
* isActive(): Returns True if the device is actively read (_read() coroutine running)
* async reset(): Resets the device if a reset_pin is given
* async stop(): Stops the device and puts it to sleep (so no new frames are received)
* async start(): Starts the device and the _read() coroutine, only needed if you called stop() before as device gets started on creation
* registerCallback(callback): register a callback or a coroutine to be called when new data is available; note that it should be short callback if you are on activeMode. On passive mode it can be as long as interval_passive_mode-2 (-45 if eco_mode is active) seconds. Also note that multiple callbacks are supported. No argument is given to the callback.
* registerEvent(event): if you did not register an event in the constructor, you can do it here
* print():  prints all read values and the time when they were read (if your time is synchronized)
* read(): returns a tuple of all values or None if the sensor is not active

The last read values (or None if the sensor is not active) can be accessed as properties:
* pm10_standard
* pm25_standard
* pm100_standard
* pm10_env
* pm25_env
* pm100_env
* particles_03um
* particles_05um
* particles_10um
* particles_25um
* particles_50um
* particles_100um
* timestamp

 
## Useful tips:
* Using Events is slower than using the callback (a few ms up to a few hundred ms, depending on how many coroutines are waiting)
* If you have a slow coroutine as callback and multiple callbacks registered, it will be faster to use Events, as callbacks are processed after each other 
* In active mode don't try to publish every frame to mqtt. It is a bit slower than getting every value and some frames get lost anyway. Building an average of every frame is possible. 
 
## Possible problems:
* setActiveMode / setPassiveMode will wait until device has been woken from sleep as these commands don't wake the device
* in active_mode uasnycio.StreamReader is used, which waits for a new byte eternally, therefore blocking the lock and preventing changes in case of error (except a reset) if the RX line stays stable. If some noise is read then the *_read()* should stop. A reset will also stop it.
