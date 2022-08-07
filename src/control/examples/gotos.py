#!/usr/bin/env python3

import asyncio
from mavsdk import System

async def print_position(drone):
    async for position in drone.telemetry.position():
        latitude = position.latitude_deg
        longitude = position.longitude_deg
        return latitude,longitude

async def run():
    drone = System()
    await drone.connect(system_address="udp://:14030")

    print("Waiting for drone to connect...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("Drone discovered!")
            break

    print("Waiting for drone to have a global position estimate...")
    async for health in drone.telemetry.health():
        if health.is_global_position_ok:
            print("Global position estimate ok")
            break

    print("Fetching amsl altitude at home location....")
    async for terrain_info in drone.telemetry.home():
        absolute_altitude = terrain_info.absolute_altitude_m
        latitude = terrain_info.latitude_deg
        longitude = terrain_info.longitude_deg
        break

    print(latitude,longitude)
    print("-- Arming")
    await drone.action.arm()

    print("-- Taking off")
    await drone.action.takeoff()
    await asyncio.sleep(4)
    while True:
        # To fly drone 20m above the ground plane
        flying_alt = absolute_altitude + 20.0
        # goto_location() takes Absolute MSL altitude
        p1 = [47.397606,8.543060]
        await drone.action.goto_location(p1[0], p1[1], flying_alt, 0)
        a = 0.0002
        latitude ,longitude = await print_position(drone)
        print(abs(latitude-p1[0])+ abs(longitude-p1[1]))
        if (abs(latitude-p1[0])+ abs(longitude-p1[1])) <= 0.0026 - a:
            print("Yaklaştı")


if __name__ == "__main__":
    loop = asyncio.get_event_loop()
    loop.run_until_complete(run())
