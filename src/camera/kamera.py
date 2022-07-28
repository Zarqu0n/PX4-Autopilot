#!/usr/bin/env python
# -*- coding: UTF-8 -*-

import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import sys


bridge = CvBridge()

def func(ros_goruntu):
  
  global bridge
  try:
    
    cv_goruntu = bridge.imgmsg_to_cv2(ros_goruntu, "bgr8")

  except CvBridgeError as e:
    print(e)
  
  cv2.imshow("Kamera", cv_goruntu)


  if cv2.waitKey(1) & 0xFF == ord('q'):
    rospy.signal_shutdown('kapatiliyor...')
  
def main(args):
  rospy.init_node('kamera', anonymous=True)

  rospy.Subscriber("/plane_cam/usb_cam/camera/image_raw",Image,func)
  
  try:
  
    rospy.spin()
  
  except KeyboardInterrupt:
    
    print("Kapatiliyor...")
    cv2.destroyAllWindows()

if __name__ == '__main__':
  main(sys.argv)