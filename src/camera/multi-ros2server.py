#!/usr/bin/env python
# -*- coding: UTF-8 -*-

import rospy
import cv2
from sensor_msgs.msg import Image,Imu
from cv_bridge import CvBridge, CvBridgeError
import sys
import socket, cv2, pickle, struct, imutils
import math

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

def callback(Imu_data):
  global teta
  global beta
  teta = math.pi/2*Imu_data.angular_velocity.y
  beta = -math.pi/2*Imu_data.angular_velocity.z

def func(ros_goruntu):
  global bridge
  data_socket = b"" 
  payload_size = struct.calcsize("Q")
  try:

      cv_goruntu = bridge.imgmsg_to_cv2(ros_goruntu, "bgr8")
      cv_goruntu = imutils.resize(cv_goruntu, width=640)
      message=[cv_goruntu,teta,beta]
      a = pickle.dumps(message)
      message = struct.pack("Q", len(a)) + a
      client_socket.sendall(message)
            
      while len(data_socket) < payload_size:
        packet = client_socket.recv(416)  # 4K
        if not packet: break
        data_socket += packet
      packed_msg_size = data_socket[:payload_size:]
      data_socket = data_socket[payload_size::]
      msg_size = struct.unpack("Q", packed_msg_size)[0]

      while len(data_socket) < msg_size:
          data_socket += client_socket.recv(416)
      dataFromServer = data_socket[:msg_size]
      data_socket = data_socket[msg_size:]
      dataFromServer= pickle.loads(dataFromServer)

      print(dataFromServer)
  except CvBridgeError as e:
      print(e)

  #cv2.imshow("Kamera", cv_goruntu)

  if cv2.waitKey(1) & 0xFF == ord('q'):
      rospy.signal_shutdown('kapatiliyor...')

def main(args):
  #dataFromServer = client_socket.recv(1024).decode()
  #print(dataFromServer)


  rospy.init_node('kamera', anonymous=True)

  rospy.Subscriber("/plane_cam0/usb_cam/camera/image_raw",Image,func)
  rospy.Subscriber("/uav0/mavros/imu/data",Imu,callback)
  try:

    rospy.spin()

  except KeyboardInterrupt:

    print("Kapatiliyor...")
    cv2.destroyAllWindows()

if __name__ == '__main__':
  main(sys.argv)
