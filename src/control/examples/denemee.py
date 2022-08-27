#!/usr/bin/env python3

import asyncio
from mavsdk import System
import math
import pytz
from datetime import datetime, timedelta, timezone

async def run():
    # Init the drone
    drone = System()
    print('aa')
    await drone.connect(system_address="serial:///dev/ttyUSB0")
    asyncio.ensure_future(print_position(drone))
    print("a")
    asyncio.ensure_future(print_imu(drone))
    asyncio.ensure_future(print_velocity(drone))
    asyncio.ensure_future(print_RawGps(drone))
    asyncio.ensure_future(print_battery(drone))

async def print_position(drone):
    async for position in drone.telemetry.position():
        lat = position.latitude_deg
        long = position.longitude_deg
        alt = position.relative_altitude_m

async def print_imu(drone):
    async for imu in drone.telemetry.attitude_euler():
        roll = imu.roll_deg
        pitch = imu.pitch_deg
        yaw = imu.yaw_deg   

async def print_velocity(drone):
    async for vel in drone.telemetry.velocity_ned():
        ground_vel = math.sqrt(vel.north_m_s**2 + vel.east_m_s**2)


async def print_RawGps(drone):
    async for raw_gps in drone.telemetry.raw_gps():
        timestamp = raw_gps.timestamp_us
        hour = timestamp//3600000000
        minute = (timestamp%3600000000)//60000000
        second = ((timestamp%3600000000)%60000000)//1000000
        millisecond = (((timestamp%3600000000)%60000000)%1000000)//1000
        print(hour,minute,second,millisecond)

async def print_battery(drone):
    async for vel in drone.telemetry.battery():
        batarya = vel.remaining_percent

if __name__ == "__main__":
    # Start the main function
    asyncio.ensure_future(run())

    # Runs the event loop until the program is canceled with e.g. CTRL-C
    asyncio.get_event_loop().run_forever()
