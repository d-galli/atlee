#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
import math
 
def euler_from_quaternion(x, y, z, w):

    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1) 

    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)

    return roll_x, pitch_y, yaw_z # in radians
    
def poseCallback(odom_message):
    
    x = odom_message.pose.pose.position.x
    y = odom_message.pose.pose.position.y

    X = odom_message.pose.pose.orientation.x
    Y = odom_message.pose.pose.orientation.y
    Z = odom_message.pose.pose.orientation.z
    W = odom_message.pose.pose.orientation.w

    eulers = euler_from_quaternion(X, Y, Z, W)
    yaw = eulers[2]

    rospy.loginfo("I heard x: %s y: %s theta: %s", x, y, yaw)

  
      
       
def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("/husky_velocity_controller/odom", Odometry, poseCallback)
    rospy.spin()
   
if __name__ == '__main__':
    listener()