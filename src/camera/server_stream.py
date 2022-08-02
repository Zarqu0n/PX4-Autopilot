#!/usr/bin/env python
# -*- coding: UTF-8 -*-

import rospy
import cv2
from sensor_msgs.msg import Image,Imu
from cv_bridge import CvBridge, CvBridgeError
import sys
import socket, cv2, pickle, struct, imutils


server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
host_name = socket.gethostname()
host_ip = '192.168.190.129'
print('HOST IP:', host_ip)
port = 5050
socket_address = (host_ip, port)

server_socket.bind(socket_address)

server_socket.listen(5)
print("LISTENING AT:", socket_address)

bridge = CvBridge()
client_socket, addr = server_socket.accept()
print('GOT CONNECTION FROM:', addr)
def func(ros_goruntu):
  global bridge
  try:

      cv_goruntu = bridge.imgmsg_to_cv2(ros_goruntu, "bgr8")
      cv_goruntu = imutils.resize(cv_goruntu, width=320)
      a = pickle.dumps(cv_goruntu)
      message = struct.pack("Q", len(a)) + a
      client_socket.sendall(message)

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
