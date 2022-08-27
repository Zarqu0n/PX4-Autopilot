#!/usr/bin/env python3

import asyncio
from turtle import pos
from mavsdk import System


async def run():
    # Init the drone
    drone = System(port=14031)
    await drone.connect(system_address="udp://:14030")
    while True:
        
        vel = await asyncio.ensure_future(print_position(drone))
        print(vel)

async def print_position(drone):
    async for position in drone.telemetry.odometry():
        return position.velocity_body.x_m_s


if __name__ == "__main__":
    asyncio.ensure_future(run())
    asyncio.get_event_loop().run_forever()
