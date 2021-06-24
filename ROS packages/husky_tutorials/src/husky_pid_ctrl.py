#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from turtlesim.msg import Pose
import math
import time
from std_srvs.srv import Empty
import numpy as np

# GLOBAL CONSTANTS #

TRESHOLD = 0.1
MAX_SPEED = 1.5
NULL_SPEED = 0.0
ANGULAR_SPEED = 1.0
LENGTH_1 = 1 # [m]
LENGTH_2 = 2 # [m]
HEADING_1 = np.pi/4
HEADING_2 = np.pi/4 + np.pi/2

# GLOBAL VARIABLES #

x = 0
y = 0
z = 0
yaw = 0
previous_time = time.time() # store initial time

previous_error = 0
Integral = 0
error = 0

state = 0




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

    x = odom_message.pose.pose.position.x
    y = odom_message.pose.pose.position.y

    X = odom_message.pose.pose.orientation.x
    Y = odom_message.pose.pose.orientation.y
    Z = odom_message.pose.pose.orientation.z
    W = odom_message.pose.pose.orientation.w

    eulers = euler_from_quaternion(X, Y, Z, W)

    yaw = eulers[2]

def PID_function(error):
    
    global Integral
    global previous_time 
    global previous_error 

    
    """ PID parameters """
    Kp = 0.6
    Ki = 0.3
    Kd = 0.2

    # Proportional component
    P_out = Kp/10 * error

    # Integral component    
    current_time = time.time()
    delta_time = current_time - previous_time  
    Integral += (error * delta_time)

    if Integral > 10:
        Integral = 10

    if Integral <-10:
        Integral =- 10

    I_out= ((Ki/10) * Integral)

    # Derivative component
    Derivative = (error/delta_time)  # de/dt
    D_out = Kd/1000 * Derivative

    # Update past values
    previous_time = current_time
    previous_error = error

    # Compute the output
    output = P_out + I_out + D_out
    
    return (output)

def moveClean():
    global state
    global x, y, yaw
    global velocity_message

    if state == 0:
        print("State: %i" % (state))

        # Set initial state
        init_pos = np.array((x, y))

        feedback = 0
        state = 1
    
    if state == 1:
        print("State: %i" % (state))
        
        set_angle = HEADING_1
        print("The desired heading angle is: %s \n" % (set_angle))

        while not rospy.is_shutdown():
            # Read the sensors
            current_yaw = yaw

            # Compute the error related to the orientation
            error = set_angle - current_yaw
            print("Current heading angle: %s\nError: %s \n" % (current_yaw, error))
            feedback = PID_function(error)
            print("Feedback: %s \n" % (feedback))

            if error > 0 :
                velocity_message.angular.z = +abs(ANGULAR_SPEED * feedback) # counterclockwise
                print("Husky rotates counterclockwise")

            else:
                velocity_message.angular.z = -abs(ANGULAR_SPEED * feedback) # clockwise
                print("Husky rotates clockwise")
            
            if abs(error) < TRESHOLD:
                velocity_message.angular.z = -abs(NULL_SPEED)
                velocity_publisher.publish(velocity_message) # publish the message to the topic

                state = 2
                break

            velocity_publisher.publish(velocity_message) # publish the message to the topic 
            loop_rate.sleep()

    if state == 2:
        print("State: %i" % (state))

        # Define the lenght of the first straight segment
        seg_len = LENGTH_1 # [m]
        
        while not rospy.is_shutdown():
            # Set initial state
            current_pos = np.array((x, y))

            # Compute Euler distance
            dist = np.linalg.norm(current_pos - init_pos)
            print("Current distance travelled : %s \n" % (dist))
        
            if dist >= seg_len:
                velocity_message.linear.x = abs(NULL_SPEED)

                feedback = 0
                state = 3
                break

            else:
                velocity_message.linear.x = abs(MAX_SPEED * 0.5) 
        
            print("Huksy moves forward")
            velocity_publisher.publish(velocity_message) # publish the message to the topic 
            loop_rate.sleep()

    if state == 3:
        print("State: %i" % (state))

        set_angle = HEADING_2
        print("The desired heading angle is: %s \n" % (set_angle))

        while not rospy.is_shutdown():

            # Read the sensors
            current_yaw = yaw

            # Compute the error related to the orientation
            error = set_angle - current_yaw
            print("Current heading angle: %s\nError: %s \n" % (current_yaw, error))
            feedback = PID_function(error)
            print("Feedback: %s \n" % (feedback))

            if error > 0 :
                velocity_message.angular.z = +abs(ANGULAR_SPEED * feedback) # counterclockwise
                print("Husky rotates counterclockwise")

            else:
                velocity_message.angular.z = -abs(ANGULAR_SPEED * feedback) # clockwise
                print("Husky rotates clockwise")

            if abs(error) < TRESHOLD:
                velocity_message.angular.z = -abs(NULL_SPEED)

                # Set initial state
                init_pos = np.array((x, y))
                state = 4
                break

            velocity_publisher.publish(velocity_message) # publish the message to the topic 
            loop_rate.sleep()

    if state == 4:
        print("State: %i" % (state))
        
        # Define the lenght of the first straight segment
        seg_len = LENGTH_2 # [m]

        while not rospy.is_shutdown():

            # Set initial state
            current_pos = np.array((x, y))

            # Compute Euler distance
            dist = np.linalg.norm(current_pos - init_pos)
            print("Current distance travelled : %s \n" % (dist))

            if dist >= seg_len:
                velocity_message.linear.x = abs(NULL_SPEED)

                feedback = 0
                state = 5
                break

            else:
                velocity_message.linear.x = abs(MAX_SPEED * 0.5)  
        
            print("Huksy moves forward")
            velocity_publisher.publish(velocity_message) # publish the message to the topic
            loop_rate.sleep()
    
    if state == 5:
        while not rospy.is_shutdown():
            print("State: %i" % (state))
            print("Final position reached!")
            velocity_message.linear.x = abs(NULL_SPEED)

            velocity_publisher.publish(velocity_message) # publish the message to the topic
            loop_rate.sleep()
            rospy.signal_shutdown("Final position reached!")

if __name__ == '__main__':
    try:
        rospy.init_node('husky_pid_controller', anonymous=True)
        loop_rate = rospy.Rate(10) # we publish the velocity at 10 Hz ( 10 times a sec)

        velocity_message = Twist() # declare a Twist message to send velocity commands
        odom_message = Odometry()

        velocity_publisher = rospy.Publisher("husky_velocity_controller/cmd_vel", Twist, queue_size=10)
        pose_subscriber = rospy.Subscriber("/husky_velocity_controller/odom", Odometry, poseCallback)
        time.sleep(2)
        
        moveClean()

    
    except rospy.ROSInterruptException:
        rospy.loginfo("Node terminated!")