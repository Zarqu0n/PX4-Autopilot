#!/usr/bin/env python3

print("\033[92m")
from ast import expr_context
import asyncio
from asyncore import file_dispatcher
from mavsdk import System
from mavsdk.geofence import Point, Polygon
from mavsdk.offboard import (OffboardError, PositionNedYaw,VelocityNedYaw,VelocityBodyYawspeed,Attitude)
import pygame
from termcolor import colored
from mavsdk.follow_me import (Config, FollowMeError, TargetLocation)
import rospy
import cv2
from sensor_msgs.msg import Image,Imu
from cv_bridge import CvBridge, CvBridgeError
import sys
import socket, cv2, pickle, struct, imutils
import math


pygame.init()
screen = pygame.display.set_mode((300, 200))
pressed = pygame.key.get_pressed()
clock = pygame.time.Clock()

#--------------------Configuration--------------------
x1,x2,y1,y2 = 0.0015,0.0015,  0.003,0.003 #Geofence distance

dist_diago_x = 0.0009#distance to geofence x
dist_diago_y = 0.00125#distance to geofence y
wp_list = list() # waypoint list
dist_var = 0.0007 # distance to waypoint

dG = 0.0008


async def check_position(drone):
    async for position in drone.telemetry.position():
        latitude = position.latitude_deg
        longitude = position.longitude_deg
        return latitude,longitude
        
async def check_flight_mode(drone):

    previous_flight_mode = None

    async for flight_mode in drone.telemetry.flight_mode():
        if flight_mode != previous_flight_mode:
            previous_flight_mode = flight_mode
            return flight_mode
            print(f"Flight mode: {flight_mode}")

async def run():

    # Connect to the Simulation
    drone = System(port=14030)
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
    gm1,gm2,gm3,gm4 = [h_latitude - x1-dG,h_longitude - y1-dG],[h_latitude + x2-dG,h_longitude - y1-dG],[h_latitude + x2-dG,h_longitude + y2+dG],[h_latitude - x1-dG, h_longitude + y2+dG]
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

    try:
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
                        c = len(changed)

                        if pressed[pygame.K_RIGHT] or pressed[pygame.K_UP] or pressed[pygame.K_LEFT]:
                            print(colored("-- Mode changed to Offboard Mode","red"))
                            await clockwise(drone,h_latitude,h_longitude,pressed)
                            print(colored("-- Continue to waypoint","red"))
                            pressed = pygame.key.get_pressed()


                    except RuntimeError as e:
                        pass
                        

            except RuntimeError as e:
                pass
        try:
            await drone.offboard.stop()
        except OffboardError as error:
            print(f"Stopping offboard mode failed with error code: \
                {error._result.result}")
    except RuntimeError as a:
        pass



async def clockwise(drone,h_latitude,h_longitude,pressed):
    for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False

    while pressed[pygame.K_RIGHT]:

        await asyncio.sleep(1)
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
        pressed = pygame.key.get_pressed()
        print(colored("-- Rotating right","green"))

        await drone.offboard.set_attitude(Attitude(30.0, 0.0, 0.0, 0.6))
        print("-- Starting offboard")
        try:
            await drone.offboard.start()
        except OffboardError as error:
            print(f"Starting offboard mode failed with error code: {error._result.result}")
            print("-- Disarming")
            await drone.action.disarm()
            return

    while pressed[pygame.K_LEFT]:

        await asyncio.sleep(1)
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
        pressed = pygame.key.get_pressed()
        print(colored("-- Rotating right","green"))
        await drone.offboard.set_attitude(Attitude(-30.0, 0.0, 0.0, 0.6))
        print("-- Starting offboard")
        try:
            await drone.offboard.start()
        except OffboardError as error:
            print(f"Starting offboard mode failed with error code: {error._result.result}")
            print("-- Disarming")
            await drone.action.disarm()
            return

    while pressed[pygame.K_UP]:
        await drone.offboard.set_attitude(Attitude(0.0, 0.0, 0.0, 0.7))
        try:
            await drone.offboard.start()
        except OffboardError as error:
            print(f"Starting offboard mode failed with error code: {error._result.result}")
            print("-- Disarming")
            await drone.action.disarm()
            return

        await asyncio.sleep(1)
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
        last_pressed = pressed
        pressed = pygame.key.get_pressed()
        changed = [idx for idx, (a, b) in enumerate(zip(last_pressed, pressed)) if a != b]
        print(colored("-- Following target","green"))

        latitude ,longitude = await check_position(drone)
        await asyncio.sleep(1)
async def follow(drone,h_latitude,h_longitude,pressed):

    for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False


    while pressed[pygame.K_UP]:
        print("-- Setting initial setpoint")
        await drone.offboard.set_velocity_body(
            VelocityBodyYawspeed(5.0, 0.0, 0.0, 0.0))
        print("-- Starting offboard")
        try:
            await drone.offboard.start()
        except OffboardError as error:
            print(f"Starting offboard mode failed with error code: {error._result.result}")
            print("-- Disarming")
            await drone.action.disarm()
            return

        await asyncio.sleep(1)
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
        last_pressed = pressed
        pressed = pygame.key.get_pressed()
        changed = [idx for idx, (a, b) in enumerate(zip(last_pressed, pressed)) if a != b]
        print(colored("-- Following target","green"))

        latitude ,longitude = await check_position(drone)
        await asyncio.sleep(1)


        wp1 = [h_latitude - x1 + dist_diago_x,h_longitude - y1 + dist_diago_y]
        wp2 = [h_latitude + x2 - dist_diago_x,h_longitude - y1 + dist_diago_y]
        wp3 = [h_latitude + x2 - dist_diago_x,h_longitude + y2 - dist_diago_y]
        wp4 = [h_latitude - x1 + dist_diago_x,h_longitude + y2 - dist_diago_y]

        """
        if latitude >= gm2[0]:
            await drone.action.goto_location(wp1[0], wp1[1], flying_alt, 0)
            print(colored("-- Approaching to upper ghost gefoence","red"))
        elif longitude <= gm2[1]:
            await drone.action.goto_location(wp2[0], wp2[1], flying_alt, 0)
            print(colored("-- Approaching to right ghost gefoence","red"))
        elif latitude <= gm1[0]:
            await drone.action.goto_location(wp3[0], wp3[1], flying_alt, 0)
            print(colored("-- Approaching to bottem ghost gefoence","red"))
        elif longitude >= gm4[1]:
            await drone.action.goto_location(wp4[0], wp4[1], flying_alt, 0)
            print(colored("-- Approaching to left ghost gefoence","red"))
        else:"""

if __name__ == "__main__":
    loop = asyncio.get_event_loop()
    loop.run_until_complete(run())
