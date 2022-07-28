#!/usr/bin/env python
# -*- coding: UTF-8 -*-

import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import sys

wCam , hCam = 680,480
wCam_int = int(wCam/4)
hCam_int = int(hCam/10)                
bridge = CvBridge()

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


  img = cv2.putText(img, '10', (360,165), cv2.FONT_HERSHEY_SIMPLEX,
                    0.5, (73,223,203), 1, cv2.LINE_AA)
  img = cv2.line(img, (325,160), (355,160), (73,223,203), 2)
  img = cv2.line(img, (335,200), (345,200), (73,223,203), 2)
  img = cv2.line(img, (325,240), (355,240), (73,223,203), 2)
  img = cv2.line(img, (335,280), (345,280), (73,223,203), 2)
  img = cv2.line(img, (325,320), (355,320), (73,223,203), 2)
  img = cv2.putText(img, '-10', (280,330), cv2.FONT_HERSHEY_SIMPLEX,
                    0.5, (73,223,203), 1, cv2.LINE_AA)

  img = cv2.line(img, (340,220), (380,260), (0,0,255), 2)
  img = cv2.line(img, (340,220), (300,260), (0,0,255), 2)

  img = cv2.line(img, (380,240), (420,240), (0,0,255), 2)
  img = cv2.line(img, (300,240), (260,240), (0,0,255), 2)


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
  
  try:
  
    rospy.spin()
  
  except KeyboardInterrupt:
    
    print("Kapatiliyor...")
    cv2.destroyAllWindows()

if __name__ == '__main__':
  main(sys.argv)