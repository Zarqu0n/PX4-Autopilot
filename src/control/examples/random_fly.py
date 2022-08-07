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
import asyncio
import random
from mavsdk import System
import asyncio
import random
  


manual_inputs = [
    [-1, 0, 0.5, 0],  # minimum roll
    [1, 0, 0.5, 0],  # maximum roll
    [0, -1, 0.5, 0],  # minimum pitch
    [0, 1, 0.5, 0],  # maximum pitch

]

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
    await asyncio.sleep(5)

    # set the manual control input after arming
    await drone.manual_control.set_manual_control_input(
        float(0), float(0), float(0.5), float(0)
    )

    # start manual control
    print("-- Starting manual control")
    await drone.manual_control.start_position_control()
    x = 0
    y = 0
    while True:



        input_index = random.choice(manual_inputs)
        input_list = input_index
        print(input_index)
        #async for position in drone.telemetry.position():
            #altitude = position.relative_altitude_m
        # get current state of roll axis (between -1 and 1)
        roll = float(input_list[0])
        # get current state of pitch axis (between -1 and 1)
        pitch = float(input_list[1])
        # get current state of throttle axis (between -1 and 1, but between 0 and 1 is expected)
        throttle = float(input_list[2])
        # get current state of yaw axis (between -1 and 1)
        yaw = float(input_list[3])
        start_time = time.time()
        seconds = 1 
        while True:
            current_time = time.time()
            elapsed_time = current_time - start_time
            await drone.manual_control.set_manual_control_input(roll, pitch, throttle, yaw)
            if elapsed_time > seconds:
                break
        await asyncio.sleep(1)

        await drone.manual_control.set_manual_control_input(0.0, 0.0, 0.5, 0.0)
        await asyncio.sleep(2)

if __name__ == "__main__":

    loop = asyncio.get_event_loop()
    loop.run_until_complete(manual_controls())
