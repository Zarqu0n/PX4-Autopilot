import cv2, imutils, socket
import numpy as np
import time
import base64
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import sys
from PIL import Image as im

bridge = CvBridge()

BUFF_SIZE = 65536
server_socket = socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
server_socket.setsockopt(socket.SOL_SOCKET,socket.SO_RCVBUF,BUFF_SIZE)
host_name = socket.gethostname()
host_ip = '192.168.190.129'#  socket.gethostbyname(host_name)
print(host_ip)
port = 6677
socket_address = (host_ip,port)
server_socket.bind(socket_address)

print('Listening at:',socket_address)


def func(ros_goruntu):

  global bridge

  try:
    
    cv_goruntu = bridge.imgmsg_to_cv2(ros_goruntu, "bgr8")
  
  except CvBridgeError as e:
    print(e)

  msg,client_addr = server_socket.recvfrom(BUFF_SIZE)

  print('GOT connection from ',client_addr)

  WIDTH=400

  frame = cv_goruntu

  frame = imutils.resize(frame,width=WIDTH)

  encoded,buffer = cv2.imencode('.jpg',frame,[cv2.IMWRITE_JPEG_QUALITY,80])

  message = base64.b64encode(buffer)

  cv2.imshow('TRANSMITTING VIDEO',frame)

  print("1")

  server_socket.sendto(message,client_addr)

  if cv2.waitKey(1) & 0xFF == ord('q'):   
    rospy.signal_shutdown('kapatiliyor...')

    cv2.destroyAllWindows()

def main(args):

    rospy.init_node('kamera', anonymous=True)

    rospy.Subscriber("/plane_cam/usb_cam/camera/image_raw",Image,func)

    rospy.spin()
  


if __name__ == '__main__':
  main(sys.argv)