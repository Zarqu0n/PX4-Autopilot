#!/usr/bin/env python3

from ast import expr_context
import asyncio
from mavsdk import System
from mavsdk.geofence import Point, Polygon
from mavsdk.offboard import (OffboardError, VelocityNedYaw)


# Test set of manual inputs. Format: [roll, pitch, throttle, yaw]

#--------------------Configuration--------------------
x1,x2,y1,y2 = 0.0015,0.0015,  0.003,0.003 #Geofence distance

dist_diago_x = 0.0009#distance to geofence x
dist_diago_y = 0.00125#distance to geofence y
wp_list = list() # waypoint list
dist_var = 0.0007 # distance to waypoint
conf_alt = 20.0


async def check_position(drone):
    async for position in drone.telemetry.position():
        latitude = position.latitude_deg
        longitude = position.longitude_deg
        return latitude,longitude
        
async def print_flight_mode(drone):

    previous_flight_mode = None

    async for flight_mode in drone.telemetry.flight_mode():
        if flight_mode != previous_flight_mode:
            previous_flight_mode = flight_mode
            print(f"Flight mode: {flight_mode}")

async def run():
    # Wait for the drone to connect
    print("Waiting for drone...")
    # Connect to the Simulation
    drone = System(port=14033)
    await drone.connect(system_address="udp://:14033")


    async for state in drone.core.connection_state():
        if state.is_connected:
            print(f"Drone discovered!")
            break

    # Fetch the home location coordinates, in order to set a boundary around the home location
    print("Fetching home location coordinates...")
    async for terrain_info in drone.telemetry.home():
        absolute_altitude = terrain_info.absolute_altitude_m
        h_latitude = terrain_info.latitude_deg
        h_longitude = terrain_info.longitude_deg
        break


    print("Old mission cleared")
    await drone.mission.clear_mission()

    # Define your geofence boundary
    
    g1 ,g2, g3, g4 = Point(h_latitude - x1, h_longitude - y1),Point(h_latitude + x2, h_longitude - y1), Point(h_latitude + x2, h_longitude + y2),Point(h_latitude - x1, h_longitude + y2)

    # Create a polygon object using your points
    polygon = Polygon([g1, g2, g3, g4], Polygon.FenceType.INCLUSION)

    px1 ,py1= h_latitude - x1 + dist_diago_x,h_longitude - y1 + dist_diago_y
    px2 ,py2= h_latitude + x2 - dist_diago_x,h_longitude - y1 + dist_diago_y
    px3 ,py3= h_latitude + x2 - dist_diago_x,h_longitude + y2 - dist_diago_y
    px4 ,py4= h_latitude - x1 + dist_diago_x,h_longitude + y2 - dist_diago_y
    px5 ,py5= h_latitude - x1 + dist_diago_x,h_longitude - y1/2 + dist_diago_y
    px6 ,py6= h_latitude + x2/2 - dist_diago_x,h_longitude - y1 + dist_diago_y
    px7 ,py7= h_latitude + x2 - dist_diago_x,h_longitude + y2/2 - dist_diago_y
    px8 ,py8= h_latitude - x1/2 + dist_diago_x,h_longitude + y2 - dist_diago_y



    # Upload the geofence to your vehicle
    print("Uploading geofence...")
    await drone.geofence.upload_geofence([polygon])

    print("Geofence uploaded!")


    print("--------- Mission Started---------")
    print("-- Arming")
    await drone.action.arm()
    print("-- Taking off")
    await asyncio.sleep(10)
    await drone.action.takeoff()
    print("-- Setting initial setpoint")
    await asyncio.sleep(10)
    await drone.offboard.set_velocity_ned(VelocityNedYaw(0.0, 0.0, 0.0, 0.0))


    while True:
        try:
            flying_alt = absolute_altitude + conf_alt
            wp1 = [px1,py1]
            wp2 = [px2,py2]
            wp3 = [px3,py3]
            wp4 = [px4,py4]
            wp5 = [px5,py5]
            wp6 = [px6,py6]
            wp7 = [px7,py7]
            wp8 = [px8,py8]

            wp_list = [wp5,wp7,wp6,wp8]
            i = 0
            print("-- Go to Waypoint{0}".format(i+1))
            while i<len(wp_list):
                wp = wp_list[i] 
                await drone.action.goto_location(wp[0], wp[1], flying_alt, 0)
                latitude ,longitude = await check_position(drone)
                if (abs(latitude-wp[0])+ abs(longitude-wp[1])) <= dist_var:
                    print("-- Reached Waypoints{0}".format(i+1))
                    i +=1
                    print("-- Going to Waypoint{0}".format(i+1))
    
        except RuntimeError as e:
            pass
        
    try:
        await drone.offboard.stop()
    except OffboardError as error:
        print(f"Stopping offboard mode failed with error code: \
              {error._result.result}")


if __name__ == "__main__":
    loop = asyncio.get_event_loop()
    loop.run_until_complete(run())
