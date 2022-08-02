#!/usr/bin/env python
# -*- coding: UTF-8 -*-

from binascii import b2a_base64
import rospy
import cv2
from sensor_msgs.msg import Image,Imu
from cv_bridge import CvBridge, CvBridgeError
import sys
import math

wCam , hCam = 680,480
wCam_int = int(wCam/4)
hCam_int = int(hCam/10)                
bridge = CvBridge()

def callback(Imu_data):
  global teta
  global beta
  teta = math.pi/2*Imu_data.angular_velocity.y
  beta = -math.pi/2*Imu_data.angular_velocity.z

def func(ros_goruntu):
  
  global bridge
  try:
    
    img = bridge.imgmsg_to_cv2(ros_goruntu, "bgr8")

  except CvBridgeError as e:
    print(e)
  
  img = cv2.resize(img, (wCam, hCam))
  img = cv2.rectangle(img,(3*wCam_int,9*hCam_int),(wCam_int,hCam_int),(73,223,203),2)
  #img = cv2.circle(img, (320,240), 160, (73,223,203), 2 )
  img = cv2.putText(img, 'FPS: 25', (300,470), cv2.FONT_HERSHEY_SIMPLEX,
                    0.5, (73,223,203), 1, cv2.LINE_AA)




  #gyro
  pi = math.pi

  cos = math.cos
  sin = math.sin
  center = [340,240]
  
  """P4----P3    center     P2----P1"""
  P2 = (int(center[0]+40*cos(teta)),int(center[1]-40*sin(teta)))
  P1 = (int(P2[0]+40*cos(teta)),int(P2[1]-40*sin(teta)))

  P3 = (int(center[0]+40*cos(pi+teta)),int(center[1]-40*sin(pi+teta)))
  P4 = (int(P3[0]+40*cos(pi+teta)),int(P3[1]-40*sin(pi+teta)))

  img = cv2.line(img, P1, P2, (0,0,255), 2)
  img = cv2.line(img, P3, P4, (255,0,255), 2)
  """F1-----F2-----F3"""
  F2 = (center[0],center[1]-20)
  F1 = (F2[0]-int(30*cos(pi+beta)),F2[1]-int(30*sin(pi+beta)))
  F3 = (F2[0]-int(30*cos(-beta)),F2[1]-int(30*sin(-beta)))
  img = cv2.line(img, F1, F2, (0,255,0), 2)
  img = cv2.line(img, F2, F3, (0,255,0), 2)
  cv2.imshow("Kamera", img)

  img = cv2.putText(img, 'Following: None', (20,60), cv2.FONT_HERSHEY_SIMPLEX,
                    0.5, (73,223,203), 1, cv2.LINE_AA)
  img = cv2.putText(img, 'Enemy Count: 0', (20,30), cv2.FONT_HERSHEY_SIMPLEX,
                    0.5, (73,223,203), 1, cv2.LINE_AA)
  img = cv2.putText(img, 'Timer: 0 sn', (20,90), cv2.FONT_HERSHEY_SIMPLEX,
                    0.5, (73,223,203), 1, cv2.LINE_AA)
  img = cv2.putText(img, 'Enemy(x): None', (20,120), cv2.FONT_HERSHEY_SIMPLEX,
                    0.5, (73,223,203), 1, cv2.LINE_AA)
  img = cv2.putText(img, 'Enemy(y): None', (20,150), cv2.FONT_HERSHEY_SIMPLEX,
                    0.5, (73,223,203), 1, cv2.LINE_AA)
  cv2.imshow("Kamera", img)


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