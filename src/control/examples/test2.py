
import rospy
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from mavros_msgs.msg import State
from sensor_msgs.msg import TimeReference
from mavros_msgs.msg import GPSRAW

def image_callback(msg):
    print(msg)

def listener():
    rospy.init_node("Velocity",anonymous=True)
    
        
    rospy.Subscriber("/mavros/gpsstatus/gps1/raw", GPSRAW, image_callback)
    rospy.spin()
    
if __name__ == '__main__':
     listener()
