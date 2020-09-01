import machine
# import pms5003
import uasyncio as asyncio
from . import pms5003

loop = asyncio.get_event_loop()

_DEFAULT_MS = 20

pm = None
lock = asyncio.Lock()


async def printit():
    pm.print()


import time


async def idle():
    while True:
        await asyncio.sleep(2)
        time.sleep_ms(5)


async def testing():
    await asyncio.sleep(120)
    while pm._sleeping_state is False:
        await asyncio.sleep_ms(100)
    print("")
    print("")
    print("----------------------------------------------------------")
    print("")
    print("Setting to active mode while sleeping")
    print("")
    print("----------------------------------------------------------")
    print("")
    print("")
    asyncio.create_task(pm.setActiveMode())
    await asyncio.sleep(120)
    print("")
    print("")
    print("----------------------------------------------------------")
    print("")
    print("Setting to passive mode whithout sleeping with 20s interval")
    print("")
    print("----------------------------------------------------------")
    print("")
    print("")
    pm.setEcoMode(False)
    asyncio.create_task(pm.setPassiveMode(20))
    await asyncio.sleep(120)
    print("")
    print("")
    print("----------------------------------------------------------")
    print("")
    print("Activating EcoMode, interval will be adapted to 60s")
    print("")
    print("----------------------------------------------------------")
    print("")
    print("")
    pm.setEcoMode(True)


def start():
    uart = machine.UART(1, tx=25, rx=26, baudrate=9600)
    global pm
    pm = pms5003.PMS5003(uart, lock, active_mode=False, interval_passive_mode=60)
    pms5003.set_debug(True)
    pm.registerCallback(printit)
    asyncio.create_task(testing())
    asyncio.get_event_loop().run_forever()


start()
