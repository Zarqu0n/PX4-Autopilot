#!/usr/bin/env python3

print("\033[92m")
from ast import expr_context
import asyncio
from mavsdk import System
from mavsdk.geofence import Point, Polygon
from mavsdk.offboard import (OffboardError, PositionNedYaw,VelocityNedYaw,VelocityBodyYawspeed)
import pygame
from termcolor import colored


pygame.init()
screen = pygame.display.set_mode((300, 200))
pressed = pygame.key.get_pressed()
clock = pygame.time.Clock()

#--------------------Configuration--------------------
x1,x2,y1,y2 = 0.002,0.002,0.004,0.004 #Geofence distance

dist_diago_x = 0.0009#distance to geofence x
dist_diago_y = 0.00125#distance to geofence y
wp_list = list() # waypoint list
dist_var = 0.0007 # distance to waypoint



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
        absolute_altitude = terrain_info.absolute_altitude_m
        h_latitude = terrain_info.latitude_deg
        h_longitude = terrain_info.longitude_deg
        break

    await asyncio.sleep(1)

    print("Old mission cleared")
    await drone.mission.clear_mission()

    # Define your geofence boundary
    
    g1 ,g2, g3, g4 = Point(h_latitude - x1, h_longitude - y1),Point(h_latitude + x2, h_longitude - y1), Point(h_latitude + x2, h_longitude + y2),Point(h_latitude - x1, h_longitude + y2)

    # Create a polygon object using your points
    polygon = Polygon([g1, g2, g3, g4], Polygon.FenceType.INCLUSION)

    # Upload the geofence to your vehicle
    print("Uploading geofence...")
    await drone.geofence.upload_geofence([polygon])

    print("Geofence uploaded!")

    print(colored("--------- Mission Started---------","yellow"))
    print("-- Arming")
    await drone.action.arm()
    print("-- Taking off")
    await drone.action.takeoff()
    print("-- Setting initial setpoint")
    await asyncio.sleep(5)
    await drone.offboard.set_velocity_ned(VelocityNedYaw(0.0, 0.0, 0.0, 0.0))


    """
    print("-- Starting offboard")
    try:
        await drone.offboard.start()
    except OffboardError as error:
        print(f"Starting offboard mode failed with error code: \
              {error._result.result}")
        print("-- Disarming")
        await drone.action.disarm()
        return"""


    while True:
        try:
            pressed = pygame.key.get_pressed()
            flying_alt = absolute_altitude + 20.0
            wp1 = [h_latitude - x1 + dist_diago_x,h_longitude - y1 + dist_diago_y]
            wp2 = [h_latitude + x2 - dist_diago_x,h_longitude - y1 + dist_diago_y]
            wp3 = [h_latitude + x2 - dist_diago_x,h_longitude + y2 - dist_diago_y]
            wp4 = [h_latitude - x1 + dist_diago_x,h_longitude + y2 - dist_diago_y]

            wp_list = [wp3,wp4,wp2,wp1]
            i = 0
            print("-- Go to Waypoint{0}".format(i+1))
            while i<4:
                try:
                    for event in pygame.event.get():
                        if event.type == pygame.QUIT:
                            running = False
                    wp = wp_list[i] 
                    await drone.action.goto_location(wp[0], wp[1], flying_alt, 0)
                    latitude ,longitude = await check_position(drone)
                    if (abs(latitude-wp[0])+ abs(longitude-wp[1])) <= dist_var:
                        print("-- Reached Waypoints{0}".format(i+1))
                        i +=1
                        print("-- Going to Waypoint{0}".format(i+1))
                    last_pressed = pressed
                    pressed = pygame.key.get_pressed()
                    changed = [idx for idx, (a, b) in enumerate(zip(last_pressed, pressed)) if a != b]
                    if len(changed) !=0:
                        print(colored(changed,"red"))
                        break
                except RuntimeError as e:
                    pass
            if i < 4:
                while len(changed) != 0:
                    """
                    await drone.manual_control.set_manual_control_input(
                    float(0), float(0), float(0.5), float(0))
                    print("-- Starting manual control")
                    await drone.manual_control.start_position_control()

                    if pressed[pygame.K_RIGHT]:
                        input_index = [0, 1, 0.5, 0]

                    elif pressed[pygame.K_LEFT]:
                        input_index = [0, -1, 0.5, 0]

                    elif pressed[pygame.K_DOWN]:
                        input_index = [1, 0, 0.5, 0]

                    elif pressed[pygame.K_UP]:
                        input_index = [-1, 0, 0.5, 0]
                    else:
                        input_index = [0, 0, 0.5, 0]
                    input_list = input_index

                    roll = float(input_list[0])
                    pitch = float(input_list[1])
                    throttle = float(input_list[2])
                    yaw = float(input_list[3])

                    await drone.manual_control.set_manual_control_input(roll, pitch, throttle, yaw)

                    await asyncio.sleep(0.1)"""
                         ####
                    print("-- Setting initial setpoint")
                    await drone.offboard.set_velocity_body(
                        VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0))

                    print("-- Starting offboard")
                    try:
                        await drone.offboard.start()
                    except OffboardError as error:
                        print(f"Starting offboard mode failed with error code: {error._result.result}")
                        print("-- Disarming")
                        await drone.action.disarm()
                        return

                    print("-- Go 0m North, 0m East, -5m Down within local coordinate system")
                    await drone.offboard.set_velocity_body(
                        VelocityBodyYawspeed(0.0, 0.0, -1.0, 60.0))
                    await asyncio.sleep(1120)

                    

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
