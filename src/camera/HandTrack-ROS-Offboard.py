#!/usr/bin/env python3

"""
This example shows how to use the manual controls plugin.

Note: Manual inputs are taken from a test set in this example to decrease complexity. Manual inputs
can be received from devices such as a joystick using third-party python extensions

Note: Taking off the drone is not necessary before enabling manual inputs. It is acceptable to send
positive throttle input to leave the ground. Takeoff is used in this example to decrease complexity
"""
import cv2
import time
import os
import HandTrackingModule as htm
import asyncio
import random
from mavsdk import System
from mavsdk.offboard import (OffboardError, PositionNedYaw,VelocityNedYaw,VelocityBodyYawspeed,Attitude)

# Test set of manual inputs. Format: [roll, pitch, throttle, yaw]
manual_inputs = [0, 0, 1, 0]  # max throttle
wCam , hCam = 640,480

cap = cv2.VideoCapture(0)

cap.set(3,wCam)
cap.set(4,hCam)

tipIds = [4,8,12,16,20]
detector = htm.handDetector(detectionCon=0.75)

async def manual_controls():
    """Main function to connect to the drone and input manual controls"""
    # Connect to the Simulation
    drone = System()
    await drone.connect(system_address="udp://:14030")

    # This waits till a mavlink based drone is connected
    async for state in drone.core.connection_state():
        if state.is_connected:
            print(f"-- Connected to drone!")
            break

    # Checking if Global Position Estimate is ok
    async for global_lock in drone.telemetry.health():
        if global_lock.is_global_position_ok:
            print("-- Global position state is ok")
            break


    # Arming the drone
    print("-- Arming")
    await drone.action.arm()

    # Takeoff the vehicle
    print("-- Taking off")
    await drone.action.takeoff()
    await asyncio.sleep(10)

    # set the manual control input after arming
    await drone.offboard.set_velocity_ned(VelocityNedYaw(0.0, 0.0, 0.0, 0.6))

    x = 0
    y = 0
    while True:
        succes,img = cap.read()
        img_hand = detector.findHands(img)
        lmlist = detector.findPosition(img_hand,draw=False)

        if len(lmlist) != 0:
            x = 0
            y = 0
            for i in range(0,20):
                x = x + lmlist[i][1]
                y = y + lmlist[i][2]
            x = x//20
            y = y//20
            img= cv2.circle(img, (x,y), 10, (0,204,204), 5)
            fingers = []
            #print(x,y)
            if lmlist[tipIds[0]][1] < lmlist[tipIds[0] - 1][1]:
                fingers.append(1)
            else:
                fingers.append(0)
            for id in range(1,5):
                if lmlist[tipIds[id]][2] < lmlist[tipIds[id]-2][2]:
                    fingers.append(1)
                else:
                    fingers.append(0)

            totalfingers = fingers.count(1)
            #print(totalfingers)
        img = cv2.rectangle(img,(480,350),(160,130),(0,204,204),2)
        img = cv2.flip(img, 1)
        cv2.imshow("Cam",img)
        cv2.waitKey(1)
        # grabs a random input from the test list
        # WARNING - your simulation vehicle may crash if its unlucky enough
        await drone.offboard.start()
        if x > 480:
            await drone.offboard.set_attitude(Attitude(30.0, 0.0, 0.0, 0.6))
        elif x < 160:
            await drone.offboard.set_attitude(Attitude(-30.0, 0.0, 0.0, 0.6))
        elif y > 350:
            await drone.offboard.set_attitude(Attitude(0.0, 0.0, 0.0, 0.6))
        elif y < 130:
            await drone.offboard.set_attitude(Attitude(0.0, 0.0, 0.0, 0.6)) 
            print("2")       
        else:
            await drone.offboard.set_attitude(Attitude(0.0, 0.0, 0.0, 0.6))
            print("1")  


        await asyncio.sleep(0.1)


if __name__ == "__main__":

    loop = asyncio.get_event_loop()
    loop.run_until_complete(manual_controls())
