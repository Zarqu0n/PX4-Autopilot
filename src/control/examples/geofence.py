#!/usr/bin/env python3

import asyncio
from mavsdk import System
from mavsdk.geofence import Point, Polygon

"""
This example shows how to use the geofence plugin.

Note: The behavior when your vehicle hits the geofence is NOT configured in this example. 

"""
x1,x2,y1,y2 = 0.0008,0.0008,  0.0020,0.0020

async def run():

    # Connect to the Simulation
    drone = System()
    await drone.connect(system_address="udp://:14030")

    # Wait for the drone to connect
    print("Waiting for drone...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print(f"Drone discovered!")
            break

    # Fetch the home location coordinates, in order to set a boundary around the home location
    print("Fetching home location coordinates...")
    async for terrain_info in drone.telemetry.home():
        latitude = terrain_info.latitude_deg
        longitude = terrain_info.longitude_deg
        break

    await asyncio.sleep(1)

    # Define your geofence boundary
    p1 = Point(latitude - x1, longitude - y1)
    p2 = Point(latitude + x2, longitude - y1)
    p3 = Point(latitude + x2, longitude + y2)
    p4 = Point(latitude - x1, longitude + y2)

    # Create a polygon object using your points
    polygon = Polygon([p1, p2, p3, p4], Polygon.FenceType.INCLUSION)

    # Upload the geofence to your vehicle
    print("Uploading geofence...")
    await drone.geofence.upload_geofence([polygon])

    print("Geofence uploaded!")


if __name__ == "__main__":
    loop = asyncio.get_event_loop()
    loop.run_until_complete(run())
