from dronekit import connect,Command,LocationGlobal
from pymavlink import mavutil
import time,sys,argparse,math

print("Bağlaniyor")
connection_sting = "serial://"
vehicle = connect(connection_sting,wait_ready = True)