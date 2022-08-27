#!/usr/bin/env python3

import asyncio
from mavsdk import System


async def run():
    # Init the drone

    
    drone = System()
    await drone.connect(system_address="serial:///dev/ttyUSB0")
    #await drone.connect(system_address = "serial:///dev/serial/by-id/usb-FTDI_FT232R_USB_UART_AB0M2AR6-if00-port0")
    #await drone.connect(system_address="serial:///dev/serial/by-id/usb-CubePilot_CubeOrange_0-if00:57600")#"udp://:14030"
    print("Bağlantı sağlandı")
    while True:
        print("Yeeee")


if __name__ == "__main__":
    # Start the main function
    asyncio.ensure_future(run())

    # Runs the event loop until the program is canceled with e.g. CTRL-C
    asyncio.get_event_loop().run_forever()
