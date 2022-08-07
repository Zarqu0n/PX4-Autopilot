#!/usr/bin/env python3


import asyncio

from mavsdk import System
from mavsdk.offboard import (OffboardError, PositionNedYaw)

async def print_flight_mode(drone):
    """ Prints the flight mode when it changes """

    previous_flight_mode = None

    async for flight_mode in drone.telemetry.flight_mode():
        if flight_mode != previous_flight_mode:
            previous_flight_mode = flight_mode
            print(f"Flight mode: {flight_mode}")


async def run():
    """ Does Offboard control using velocity NED coordinates. """

    drone = System()
    await drone.connect(system_address="udp://:14030")

    print("Waiting for drone to connect...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print(f"Drone discovered!")
            break

    print("-- Arming")
    await drone.action.arm()
    print("-- Taking off")
    await drone.action.takeoff()
    print("-- Setting initial setpoint")
    await asyncio.sleep(10)
    await drone.offboard.set_velocity_ned(VelocityNedYaw(0.0, 0.0, 0.0, 0.0))

    print("-- Starting offboard")
    try:
        await drone.offboard.start()
    except OffboardError as error:
        print(f"Starting offboard mode failed with error code: \
              {error._result.result}")
        print("-- Disarming")
        await drone.action.disarm()
        return
    
    print_flight_mode_task = asyncio.ensure_future(print_flight_mode(drone))
    print("-- Go up 2 m/s")
    await drone.offboard.set_velocity_ned(VelocityNedYaw(0.0, 5.0, 5.0, 0.0))
    await asyncio.sleep(10)
    print_flight_mode_task = asyncio.ensure_future(print_flight_mode(drone))
    print("-- Go North 2 m/s, turn to face East")
    await drone.offboard.set_velocity_ned(VelocityNedYaw(2.0, 0.0, 2.0, 90.0))
    await asyncio.sleep(10)
    print_flight_mode_task = asyncio.ensure_future(print_flight_mode(drone))
    print("-- Go South 2 m/s, turn to face West")
    await drone.offboard.set_velocity_ned(
        VelocityNedYaw(-2.0, 0.0, 0.0, 270.0))
    await asyncio.sleep(10)
    print_flight_mode_task = asyncio.ensure_future(print_flight_mode(drone))
    print("-- Go West 2 m/s, turn to face East")
    await drone.offboard.set_velocity_ned(VelocityNedYaw(0.0, -2.0, 2.0, 90.0))
    await asyncio.sleep(10)
    print_flight_mode_task = asyncio.ensure_future(print_flight_mode(drone))
    print("-- Go East 2 m/s")
    await drone.offboard.set_velocity_ned(VelocityNedYaw(0.0, 2.0, 2.0, 90.0))
    await asyncio.sleep(10)
    print_flight_mode_task = asyncio.ensure_future(print_flight_mode(drone))
    print("-- Turn to face South")
    await drone.offboard.set_velocity_ned(VelocityNedYaw(0.0, 0.0, 2.0, 180.0))
    await asyncio.sleep(10)
    print_flight_mode_task = asyncio.ensure_future(print_flight_mode(drone))
    print("-- Go down 1 m/s, turn to face North")
    await drone.offboard.set_velocity_ned(VelocityNedYaw(0.0, 0.0, 1.0, 0.0))
    await asyncio.sleep(10)
    print_flight_mode_task = asyncio.ensure_future(print_flight_mode(drone))
    print("-- Stopping offboard")
    try:
        await drone.offboard.stop()
    except OffboardError as error:
        print(f"Stopping offboard mode failed with error code: \
              {error._result.result}")


if __name__ == "__main__":
    loop = asyncio.get_event_loop()
    loop.run_until_complete(run())
