#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from turtlesim.msg import Pose
import math
import time
from std_srvs.srv import Empty

x = 0
y = 0
z = 0
yaw = 0

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
    global x, y, z, yaw
    x = odom_message.PoseWithCovariance.Pose.Point.x
    y = odom_message.PoseWithCovariance.Pose.Point.y

    X = odom_message.PoseWithCovariance.Pose.Quaternion.x
    Y = odom_message.PoseWithCovariance.Pose.Quaternion.y
    Z = odom_message.PoseWithCovariance.Pose.Quaternion.z
    W = odom_message.PoseWithCovariance.Pose.Quaternion.w

    eulers = euler_from_quaternion(X, Y, Z, W)

    yaw = eulers[2]

def stop():
    global velocity_message

   # reset the velocity message 
    velocity_message = Twist()
    velocity_message.linear.x = 0
    velocity_message.linear.y = 0
    velocity_message.linear.z = 0
    velocity_message.angular.x = 0
    velocity_message.angular.y = 0
    velocity_message.angular.z = 0

    velocity_publisher.publish(velocity_message) # publish the message to the topic 

def move (speed, distance, is_forward):
    # get current location
    global x, y, velocity_message
    x_0 = x
    y_0 = y

    # reset the travel distance 
    travel_distance = 0.0 

    # reset the velocity message 
    velocity_message.linear.x = 0
    velocity_message.linear.y = 0
    velocity_message.linear.z = 0
    velocity_message.angular.x = 0
    velocity_message.angular.y = 0
    velocity_message.angular.z = 0

    # check the direction of motion 
    if (is_forward):
        velocity_message.linear.x = abs(speed)
    else:
        velocity_message.linear.x = -abs(speed)
    
    rospy.loginfo("Robots moves forward")

    while not rospy.is_shutdown():
        
        velocity_publisher.publish(velocity_message) # publish the message to the topic 
        loop_rate.sleep()

        travel_distance = travel_distance + math.sqrt(((x - x_0)**2) + ((y - y_0)**2)) # compute the travelled distance as the Euclidean's distance 
        # print(travel_distance)
        if (travel_distance >= distance):
            rospy.loginfo("Goal reached")
            break
    
    # stop the motion 
    velocity_message.linear.x = 0
    velocity_message.linear.y = 0
    velocity_message.linear.z = 0
    velocity_message.angular.x = 0
    velocity_message.angular.y = 0
    velocity_message.angular.z = 0
    velocity_publisher.publish(velocity_message)

def rotate (angular_speed_deg, relative_angle_deg, clockwise):
    global yaw, loop_rate, velocity_publisher, velocity_message

    # reset the velocity message 
    velocity_message.linear.x = 0
    velocity_message.linear.y = 0
    velocity_message.linear.z = 0
    velocity_message.angular.x = 0
    velocity_message.angular.y = 0
    velocity_message.angular.z = 0

    # get current location
    theta_0 = yaw
    angular_speed = math.radians(abs(angular_speed_deg))

    # check the direction of rotation
    if (clockwise):
        velocity_message.angular.z = -abs(angular_speed)
    else:
        velocity_message.angular.z = abs(angular_speed)
    
   # set the travelled angle to zero 
    start_angle = 0.0
    t_0 = rospy.Time.now().to_sec()# store current time value

    rospy.loginfo("Turtlesim rotates")

    while not rospy.is_shutdown():
        
        velocity_publisher.publish(velocity_message) # publish the message to the topic

        t_1 = rospy.Time.now().to_sec() # store the current time value 
        final_angle = (t_1 - t_0) * angular_speed
        loop_rate.sleep()

        if (final_angle >= math.radians(relative_angle_deg)):
            rospy.loginfo("Reached!")
            break

    # reset the velocity message 
    velocity_message.linear.x = 0
    velocity_message.linear.y = 0
    velocity_message.linear.z = 0
    velocity_message.angular.x = 0
    velocity_message.angular.y = 0
    velocity_message.angular.z = 0
    velocity_publisher.publish(velocity_message) # publish the message to the topic


def squareClean():
    
    distance = float(input("Please, enter the lenght of the edges in meterspp:\n"))

    move(1.0, distance, True) # speed, lenght, is_forward 
    rotate(20, 90, False)
    move(1.0, distance, True)
    rotate(20, 180, False)
    move(1.0, distance, True)
    rotate(20, 90, True)
    move(1.0, distance, True)

    pass

if __name__ == '__main__':
    try:
        rospy.init_node('motion_control', anonymous=True)
        loop_rate = rospy.Rate(10) # we publish the velocity at 10 Hz ( 10 times a sec)

        velocity_message = Twist() # declare a Twist message to send velocity commands
        odom_message = Odometry()

        velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        pose_subscriber = rospy.Subscriber("husky_velocity_controller/odom", Pose, poseCallback)
        time.sleep(2)

       # user_interface()
        squareClean()

    
    except rospy.ROSInterruptException:
        rospy.loginfo("Node terminated!")