#!/usr/bin/env python
# -*- coding: UTF-8 -*-

from re import sub
from tkinter import CENTER
from tokenize import String
import rospy
import cv2
from sensor_msgs.msg import Image
from sensor_msgs.msg import Imu
from cv_bridge import CvBridge, CvBridgeError
import sys
import math

bridge = CvBridge()

def callback(Imu_data):
  global teta
  global beta
  teta = math.pi/2*Imu_data.angular_velocity.y
  beta = -math.pi/2*Imu_data.angular_velocity.z
def func(ros_goruntu):
  
  global bridge
  try:
    
    cv_goruntu = bridge.imgmsg_to_cv2(ros_goruntu, "bgr8")

  except CvBridgeError as e:
    print(e)


   #gyro
  pi = math.pi

  cos = math.cos
  sin = math.sin
  center = [340,240]

  """P4----P3    center     P2----P1"""
  P2 = (int(center[0]+40*cos(teta)),int(center[1]-40*sin(teta)))
  P1 = (int(P2[0]+40*cos(teta)),int(P2[1]-40*sin(teta)))#Right

  P3 = (int(center[0]+40*cos(pi+teta)),int(center[1]-40*sin(pi+teta)))
  P4 = (int(P3[0]+40*cos(pi+teta)),int(P3[1]-40*sin(pi+teta)))

  cv_goruntu = cv2.line(cv_goruntu, P1, P2, (0,0,255), 2)#sağ c-d
  cv_goruntu = cv2.line(cv_goruntu, P3, P4, (255,0,255), 2)#sol b-a
  """F1-----F2-----F3"""
  F2 = (center[0],center[1]-40)
  F1 = (F2[0]-int(20*cos(pi+beta)),F2[1]-int(20*sin(pi+beta)))
  F3 = (F2[0]-int(20*cos(-beta)),F2[1]-int(20*sin(-beta)))
  cv_goruntu = cv2.line(cv_goruntu, F1, F2, (0,255,0), 2)#sağ c-d
  cv_goruntu = cv2.line(cv_goruntu, F2, F3, (0,255,0), 2)#sol b-a
  cv2.imshow("Kamera", cv_goruntu)




  if cv2.waitKey(1) & 0xFF == ord('q'):
    rospy.signal_shutdown('kapatiliyor...')
  
def main(args):
  rospy.init_node('kamera', anonymous=True)

  rospy.Subscriber("/plane_cam/usb_cam/camera/image_raw",Image,func)  
  rospy.Subscriber("/mavros/imu/data",Imu,callback)
  try:
  
    rospy.spin()
  
  except KeyboardInterrupt:
    
    print("Kapatiliyor...")
    cv2.destroyAllWindows()

if __name__ == '__main__':
  main(sys.argv)