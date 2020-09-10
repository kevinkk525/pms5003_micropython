# MIT License
#
# Copyright (c) 2018-2020, Kevin Köck
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.

# based on circuitpython-code of adafruit: https://learn.adafruit.com/pm25-air-quality-sensor/circuitpython-code
# and some inspiration from https://github.com/RigacciOrg/AirPi/blob/master/lib/pms5003

# command responses are only processed generally but can't distinguish between different responses

__updated__ = "2020-07-04"
__version__ = "1.9.12"

import uasyncio as asyncio
import time

try:
    import struct
except ImportError:
    import ustruct as struct

# Sensor settling after wakeup requires at least 30 seconds (sensor sepcifications).
WAIT_AFTER_WAKEUP = 40

# Normal data frame length.
DATA_FRAME_LENGTH = 28
# Command response frame length.
CMD_FRAME_LENGTH = 4

# Maximum tries after which the device is being reset, the actual tries are twice as much as library tries 2 times
MAX_COMMAND_FAILS = 3

DEBUG = False


def set_debug(debug):
    global DEBUG
    DEBUG = debug


class PMS5003_base:
    def __init__(self, uart, lock=None, set_pin=None, reset_pin=None, interval_passive_mode=None,
                 event=None, active_mode=True, eco_mode=True, assume_sleeping=True):
        self._uart = uart  # accepts a uart object
        self._set_pin = set_pin
        if set_pin is not None:
            set_pin.value(1)
        self._reset_pin = reset_pin
        if reset_pin is not None:
            reset_pin.value(1)
        self._active = True
        self._active_mode = active_mode  # passive mode will be set on first wakeUp() in _read()
        self._eco_mode = eco_mode  # only works with passive mode as sleep is not possible in active_mode
        self._sreader = asyncio.StreamReader(uart)
        self._interval_passive_mode = interval_passive_mode or 60  # in case someone forgets to set it
        if self._eco_mode and self._active_mode is False and self._interval_passive_mode < WAIT_AFTER_WAKEUP + 5:
            self._error(
                "interval_passive_mode can't be less than DEVICE_WAKEUP_TIME of {!s}s".format(
                    WAIT_AFTER_WAKEUP + 5))
            self._interval_passive_mode = 60
        self._event = event
        self._lock = asyncio.Lock()
        self._timestamp = None
        self._sleeping_state = assume_sleeping  # assume sleeping on start by default
        self._invalidateMeasurements()
        self._callback = None  # can be a short coroutine too; no args given
        asyncio.create_task(self._read())

    @staticmethod
    def _error(message):
        # Default logging implementation, to be overriden in subclasses if logging required
        print(message)

    @staticmethod
    def _warn(message):
        # Default logging implementation, to be overriden in subclasses if logging required
        print(message)

    @staticmethod
    def _debug(message):
        # Default logging implementation, to be overriden in subclasses if logging required
        if DEBUG:
            print(message)

    def setEcoMode(self, value=True):
        """Puts device to sleep between readings in passive mode"""
        self._eco_mode = value
        if self._eco_mode and self._active_mode is False and self._interval_passive_mode < WAIT_AFTER_WAKEUP + 5:
            self._error(
                "interval_passive_mode can't be less than DEVICE_WAKEUP_TIME of {!s}s".format(
                    WAIT_AFTER_WAKEUP + 5))
            self._interval_passive_mode = 60

    async def setActiveMode(self):
        if self._active is False:
            self._active_mode = True
            return True
        self._debug("setActiveMode")
        async with self._lock:
            self._debug("setActiveMode got lock")
            res = await self._sendCommand(0xe1, 0x01)
            if res is None:
                await asyncio.sleep_ms(100)
                res = await self._sendCommand(0xe1, 0x01)
                if res is None:
                    self._error("Error putting device in active mode")
                    return False
            self._active_mode = True
            self._debug("setActiveMode Done")
        return True

    async def setPassiveMode(self, interval=None):
        if self._active is False:
            self._active_mode = False
            return True
        self._debug("setPassiveMode")
        async with self._lock:
            self._debug("setPassiveMode got lock")
            res = await self._sendCommand(0xe1, 0x00)
            if res is None:
                await asyncio.sleep_ms(100)
                res = await self._sendCommand(0xe1, 0x00)
                if res is None:
                    self._error("Error putting device in passive mode")
                    return False
            if interval is not None:
                self._interval_passive_mode = interval
                if self._eco_mode and self._active_mode is False and self._interval_passive_mode < WAIT_AFTER_WAKEUP + 5:
                    self._error(
                        "interval_passive_mode can't be less than DEVICE_WAKEUP_TIME of {!s}s".format(
                            WAIT_AFTER_WAKEUP + 5))
                    self._interval_passive_mode = 60
            self._active_mode = False
            await asyncio.sleep_ms(100)
            self._flush_uart()  # no leftovers from active mode
        self._debug("setPassiveMode done")
        return True

    async def sleep(self):
        self._debug("sleep")
        async with self._lock:
            self._debug("sleep got lock")
            if self._set_pin is not None:
                self._set_pin.value(0)
                # response on pin change?
            else:
                res = await self._sendCommand(0xe4, 0x00)
                if res is None:
                    await asyncio.sleep_ms(100)
                    res = await self._sendCommand(0xe4, 0x00)
                    if res is None:
                        self._sleeping_state = True  # just to make it possible for wakeUp to try again
                        self._error("Error putting device to sleep")
                        return False
            self._sleeping_state = True
        self._debug("Putting device to sleep")
        return True

    async def wakeUp(self):
        self._debug("wakeUp")
        async with self._lock:
            self._debug("wakeUp got lock")
            self._flush_uart()
            if self._set_pin is not None:
                self._set_pin.value(1)
                self._debug("Waiting {!s}s".format(WAIT_AFTER_WAKEUP))
                await asyncio.sleep_ms(WAIT_AFTER_WAKEUP)
                self._flush_uart()
                res = await self._read_frame()
                if res is None:
                    self._error("No response to wakeup pin change")
                    return False
            else:
                res = await self._sendCommand(0xe4, 0x01, False, delay=16000,
                                              wait=WAIT_AFTER_WAKEUP * 1000)
                if res is None:
                    await asyncio.sleep_ms(100)
                    res = await self._sendCommand(0xe4, 0x01, False, delay=16000,
                                                  wait=WAIT_AFTER_WAKEUP * 1000)
                    if res is None:
                        self._error("No response to wakeup command")
                        return False
                self._flush_uart()
        self._debug("device woke up")
        self._sleeping_state = False
        if self._active_mode is False:
            if self._lock.locked():
                self._error("Lock should be released in wakeUp before setPassiveMode")
            await self.setPassiveMode()
            # device does not save passive state
        self._debug("wakeUp done")
        return True

    def isActive(self):
        return self._active

    async def reset(self):
        if self._reset_pin is not None:
            self._reset_pin.value(0)
            await asyncio.sleep(5)
            self._reset_pin.value(1)
            if self._active:
                async with self._lock:
                    if await self._read_frame() is None:
                        self._error("Reset did not work, reset manually")
                        return False
                    return True
        else:
            self._error("No reset pin defined, can't reset")
            return False

    async def _sendCommand(self, command, data, expect_command=True, delay=1000, wait=None):
        self._debug(
            "Sending command: {!s},{!s},{!s},{!s}".format(command, data, expect_command, delay))
        arr = bytearray(7)
        arr[0] = 0x42
        arr[1] = 0x4d
        arr[2] = command
        arr[3] = 0x00
        arr[4] = data
        s = sum(arr[:5])
        arr[5] = int(s / 256)
        arr[6] = s % 256
        self._flush_uart()
        self._uart.write(arr)
        et = time.ticks_ms() + delay + (wait if wait else 0)
        frame_len = CMD_FRAME_LENGTH + 4 if expect_command else DATA_FRAME_LENGTH + 4
        # self._debug("Expecting {!s}".format(frame_len))
        if wait:
            self._debug("waiting {!s}s".format(wait / 1000))
            await asyncio.sleep_ms(wait)
            self._flush_uart()
        while time.ticks_ms() < et:
            await asyncio.sleep_ms(100)
            if self._uart.any() >= frame_len:
                # going through all pending data frames
                res = await self._read_frame()
                if res is True and expect_command:
                    self._debug("Got True")
                    return True
                elif res is not None:
                    self._debug("Got {!s}".format(res))
                    return res
                else:
                    pass  # try again until found a valid one or timeout
                await asyncio.sleep_ms(100)
        self._debug("Got no available bytes")
        return None

    async def stop(self):
        self._active = False
        await self.sleep()

    async def start(self):
        # coroutine as everything else is a coroutine
        if self._active is False:
            self._active = True
            asyncio.create_task(self._read())
        else:
            self._warn("Sensor already active")

    def registerCallback(self, callback):
        # callback will be called on every new sensor value, should be fast in active mode
        if self._callback is None:
            self._callback = callback
        elif type(self._callback) == list:
            self._callback.append(callback)
        else:
            self._callback = [self._callback, callback]

    def registerEvent(self, event):
        # enhances usability; by using an event with active mode a fast reaction to
        # changing values is possible
        self._event = event

    def print(self):
        if self._active and self._timestamp is not None:
            print("")
            print("---------------------------------------------")
            t = time.localtime()
            print("Measurement {!s}ms ago at {}-{:02d}-{:02d} {:02d}:{:02d}:{:02d}".format(
                time.ticks_ms() - self._timestamp, t[0], t[1], t[2], t[3], t[4], t[5]))
            print("---------------------------------------------")
            print("Concentration Units (standard)")
            print("---------------------------------------------")
            print("PM 1.0: %d\tPM2.5: %d\tPM10: %d" % (
                self._pm10_standard, self._pm25_standard, self._pm100_standard))
            print("Concentration Units (environmental)")
            print("---------------------------------------------")
            print("PM 1.0: %d\tPM2.5: %d\tPM10: %d" % (
                self._pm10_env, self._pm25_env, self._pm100_env))
            print("---------------------------------------------")
            print("Particles > 0.3um / 0.1L air:", self._particles_03um)
            print("Particles > 0.5um / 0.1L air:", self._particles_05um)
            print("Particles > 1.0um / 0.1L air:", self._particles_10um)
            print("Particles > 2.5um / 0.1L air:", self._particles_25um)
            print("Particles > 5.0um / 0.1L air:", self._particles_50um)
            print("Particles > 10 um / 0.1L air:", self._particles_100um)
            print("---------------------------------------------")
            print("")
        else:
            print("PMS5003 Sensor not active")

    def _flush_uart(self):
        while self._uart.any():
            self._uart.read(self._uart.any())

    async def _read(self):
        woke_up = None
        if self._sleeping_state:
            await self.wakeUp()  # just in case controller rebooted and left device in sleep mode
            woke_up = time.ticks_ms()
        elif self._active_mode is False:
            await self.setPassiveMode()
        last_reading = time.ticks_ms()
        while self._active:
            diff = time.ticks_ms() - last_reading
            if woke_up is not None:
                diff = time.ticks_ms() - woke_up
            if self._active_mode and diff > 60000:
                # in passive mode it would be detected when requesting new data
                # but maybe this does not work because of StreamReader
                self._warn("No new data since 60s, resetting device")
                if await self.reset() is False:
                    self._error("Disabling device as it can not be reset")
                    self._active = False
                    self._sleeping_state = True
                    # always assume sleeping state if device is started again as this also sets active_mode
            if self._sleeping_state is False:  # safeguard as wakeUp() while inside lock is not possible
                async with self._lock:
                    self._debug("_read got lock")
                    frame = None
                    counter = 0
                    while frame is None and counter < 5:
                        if self._active_mode:
                            frame = await self._read_frame(False, True)  # lock already acquired
                        else:
                            frame = await self._sendCommand(0xe2, 0x00, False, delay=10000)
                        if frame is not None:
                            self._pm10_standard = frame[0]
                            self._pm25_standard = frame[1]
                            self._pm100_standard = frame[2]
                            self._pm10_env = frame[3]
                            self._pm25_env = frame[4]
                            self._pm100_env = frame[5]
                            self._particles_03um = frame[6]
                            self._particles_05um = frame[7]
                            self._particles_10um = frame[8]
                            self._particles_25um = frame[9]
                            self._particles_50um = frame[10]
                            self._particles_100um = frame[11]
                            self._timestamp = time.ticks_ms()
                            if self._active and self._event is not None:
                                self._event.set()
                            if self._active and self._callback is not None:
                                cbs = [self._callback] if type(
                                    self._callback) != list else self._callback
                                for cb in cbs:
                                    # call callback or await coroutine, should be short.
                                    tmp = cb()
                                    if str(type(tmp)) == "<class 'generator'>":
                                        await tmp
                            last_reading = time.ticks_ms()
                        counter += 1
                        await asyncio.sleep_ms(100)
            if self._active_mode:
                await asyncio.sleep_ms(100)
                # give other commands time to send and receive response (keep lock free)
                woke_up = None
            else:
                sleep = self._interval_passive_mode - (time.ticks_ms() - last_reading) / 1000
                if self._eco_mode:
                    await self.sleep()
                    sleep -= (WAIT_AFTER_WAKEUP + 1)
                    # +1 is experience as commands to wakeup and set mode take time
                else:
                    woke_up = None  # probably changed mode during sleep
                if sleep < 2:  # making 2s between reading attempts the smallest interval
                    sleep = 2
                sleep = int(sleep)
                # a bit of rounding the value, a few hundred ms earlier is needed for commands
                self._debug("loop sleep for {!s}s".format(sleep))
                await asyncio.sleep(sleep)
                if self._sleeping_state:
                    await self.wakeUp()
                    woke_up = time.ticks_ms()

        self._invalidateMeasurements()  # set values to None if device is not active anymore

    async def _read_frame(self, with_lock=False, with_async=False):
        # using lock to prevent multiple coroutines from reading at the same time
        self._debug("readFrame {!s} {!s}".format(with_lock, with_async))
        if with_lock:
            async with self._lock:
                self._debug("readFrame got lock")
                res = await self.__read_frame(with_async)  # can be None
                self._debug("readFrame got: {!s}".format(res))
                return res
        else:
            res = await self.__read_frame(with_async)  # can be None
            self._debug("readFrame got: {!s}".format(res))
            return res

    async def __await_bytes(self, count, timeout):
        st = time.ticks_ms()
        while self._uart.any() < count:
            await asyncio.sleep_ms(20)
            if time.ticks_ms() - st > timeout:
                return

    async def __read_frame(self, with_async):
        buffer = []
        start = time.ticks_ms()
        timeout = 200
        self._debug("__read_frame")
        available = self._uart.any()
        if available > 32 and available % 32 == 0:
            self._uart.read(available - 32)  # just throw away the oldest data_frames
            self._debug("Throwing away old data_frames, #bytes {!s}".format(available - 32))
        while True:
            if with_async is False and time.ticks_ms() - start > timeout:
                self._debug(
                    "Reading a lot of noise on RX line to exceed timeout of {!s}ms, availble bytes {!s}".format(
                        timeout, self._uart.any()))
                return None
            preframe_len = 4 + CMD_FRAME_LENGTH - len(buffer)
            if with_async:
                # StreamReader seems to have problems reading the correct amount of bytes
                data = b""
                count = 0
                while len(data) < preframe_len:
                    data += await self._sreader.read(preframe_len - len(data))
                    if count > 5:
                        break
                    count += 1
            else:
                await self.__await_bytes(preframe_len, 100)
                data = self._uart.read(preframe_len)
            if len(data) is None:
                self._debug("Read no data from uart despite having waited for data")
                return None
            if len(data) != preframe_len and len(data) > 0:
                self._error(
                    "Short read, expected {!s} bytes, got {!s}".format(preframe_len, len(data)))
                return None
            if data == b'':
                return None
            buffer += list(data)
            while len(buffer) >= 2 and buffer[0] != 0x42 and buffer[1] != 0x4d:
                buffer.pop(0)
            if len(buffer) == 0:
                continue
            elif len(buffer) < 4:
                continue
            frame_len = struct.unpack(">H", bytes(buffer[2:4]))[0]
            if frame_len == DATA_FRAME_LENGTH:
                if with_async:
                    # StreamReader seems to have problems reading the correct amount of bytes
                    data = b""
                    count = 0
                    while len(data) < frame_len - CMD_FRAME_LENGTH:
                        data += await self._sreader.read(frame_len - CMD_FRAME_LENGTH - len(data))
                        if count > 5:
                            break
                        count += 1
                else:
                    await self.__await_bytes(frame_len - CMD_FRAME_LENGTH, 100)
                    data = self._uart.read(frame_len - CMD_FRAME_LENGTH)
                if len(data) != DATA_FRAME_LENGTH - CMD_FRAME_LENGTH:
                    self._error(
                        "Short read, expected {!s} bytes, got {!s}".format(frame_len, len(data)))
                    return None
                buffer += list(data)
                check = buffer[-2] * 256 + buffer[-1]
                checksum = sum(buffer[0:frame_len + 2])
                if check == checksum:
                    if self._uart.any() > 32:
                        self._flush_uart()  # just to prevent getting flooded if a callback took too long
                        self._warn("Getting too many new data frames, callback too slow")
                    frame = struct.unpack(">HHHHHHHHHHHHHH", bytes(buffer[4:]))
                    no_values = True
                    for i in range(6, 12):
                        if frame[i] != 0:
                            no_values = False
                    if no_values:
                        buffer = []
                        self._debug("got no values")
                        await asyncio.sleep_ms(50)
                        start = time.ticks_ms()  # reset timeout counter
                        continue
                    else:
                        return frame
            elif frame_len == CMD_FRAME_LENGTH:
                check = buffer[-2] * 256 + buffer[-1]
                checksum = sum(buffer[0:frame_len + 2])
                if check == checksum:
                    self._debug("Received command response frame: {!s}".format(buffer))
                    return True
                else:
                    return None
            elif frame_len == 0:
                pass  # wrong frame/bytes received
            else:
                self._warn("Unexpected frame_len {!s}, probably random or scrambled bytes".format(
                    frame_len))

            buffer = []
            continue

            # pm10_standard, pm25_standard, pm100_standard, pm10_env,
            # pm25_env, pm100_env, particles_03um, particles_05um, particles_10um,
            # particles_25um, particles_50um, particles100um, skip, checksum=frame

    def _invalidateMeasurements(self):
        self._pm10_standard = None
        self._pm25_standard = None
        self._pm100_standard = None
        self._pm10_env = None
        self._pm25_env = None
        self._pm100_env = None
        self._particles_03um = None
        self._particles_05um = None
        self._particles_10um = None
        self._particles_25um = None
        self._particles_50um = None
        self._particles_100um = None

    @property
    def pm10_standard(self):
        return self._pm10_standard

    @property
    def pm25_standard(self):
        return self._pm25_standard

    @property
    def pm100_standard(self):
        return self._pm100_standard

    @property
    def pm10_env(self):
        return self._pm10_env

    @property
    def pm25_env(self):
        return self._pm25_env

    @property
    def pm100_env(self):
        return self._pm100_env

    @property
    def particles_03um(self):
        return self._particles_03um

    @property
    def particles_05um(self):
        return self._particles_05um

    @property
    def particles_10um(self):
        return self._particles_10um

    @property
    def particles_25um(self):
        return self._particles_25um

    @property
    def particles_50um(self):
        return self._particles_50um

    @property
    def particles_100um(self):
        return self._particles_100um

    def read(self):
        if self._active:
            return (self._pm10_standard, self._pm25_standard, self._pm100_standard,
                    self._pm10_env, self._pm25_env, self._pm100_env,
                    self._particles_03um, self._particles_05um, self._particles_10um,
                    self._particles_25um, self._particles_50um, self._particles_100um)
        return None

    @property
    def timestamp(self):
        return self._timestamp


class PMS5003(PMS5003_base):
    def __init__(self, uart, lock=None, set_pin=None, reset_pin=None, interval_passive_mode=None,
                 event=None, active_mode=True, eco_mode=True, assume_sleeping=True):
        super().__init__(uart, set_pin=set_pin, reset_pin=reset_pin,
                         interval_passive_mode=interval_passive_mode,
                         event=event, active_mode=active_mode, eco_mode=eco_mode,
                         assume_sleeping=assume_sleeping)

    async def _makeResilient(self, *args, **kwargs):
        if "first_try" not in kwargs:
            first_try = True
        else:
            first_try = kwargs["first_try"]
            del kwargs["first_try"]
        if "command" in kwargs:
            command = kwargs["command"]
            del kwargs["command"]
        else:
            command = args[0]
            args = args[1:]
        count = 0
        while count < MAX_COMMAND_FAILS:
            if await command(*args, **kwargs) is False:
                count += 1
                await asyncio.sleep(1)
            else:
                return True
        if first_try:
            self._warn("Resetting not responding device")
            if await self.reset() is False:
                self._error("Shutting device down as it is not responding")
                self._active = False
                return False
        else:
            self._error("Shutting device down as it responds wrong even after reset")
            self._active = False
            self._sleeping_state = True
            # always assume sleeping state if device is started again as this also sets active_mode
            return False
        args = (command,) + args
        kwargs["first_try"] = False
        await self._makeResilient(*args, **kwargs)

    async def wakeUp(self):
        await self._makeResilient(super().wakeUp)

    async def sleep(self):
        await self._makeResilient(super().sleep)

    async def setActiveMode(self):
        while self._active is True and self._sleeping_state is True:
            await asyncio.sleep_ms(100)
            # device has to wake up first and after that we'll set the state or
            # weird behaviour possible otherwise
        await self._makeResilient(super().setActiveMode)

    async def setPassiveMode(self, interval=None):
        while self._active is True and self._sleeping_state is True:
            await asyncio.sleep_ms(100)
            # device has to wake up first and after that we'll set the state or
            # weird behaviour possible otherwise
        await self._makeResilient(super().setPassiveMode, interval=interval)
