#!/usr/bin/env python
#EXPLANATION: Python script to test Franka in joint space

import rospy, os, sys
import numpy as np
#import collision_check_toolbox as ccToolBox
import tf

import time
#import matrix_tool as matrix_tool
import math

from trajectory_msgs.msg import JointTrajectoryPoint, JointTrajectory
from geometry_msgs.msg import Twist, TwistStamped, Quaternion
from control_msgs.msg import FollowJointTrajectoryGoal, FollowJointTrajectoryActionGoal, JointTolerance
from sensor_msgs.msg import JointState
from franka_control.msg import ErrorRecoveryActionGoal
from franka_msgs.msg import FrankaState
from franka_gripper.msg import GraspActionGoal, MoveActionGoal
from trac_ik_python.trac_ik import IK
from controller_manager_msgs.srv import UnloadController, LoadController, SwitchController
import controller_manager_msgs.msg

from std_msgs.msg import Float32MultiArray
from std_msgs.msg import String, Bool
from std_msgs.msg import Float32

import ast

# inside the class all the functions used in the listener function are declared
class Arm:
    def __init__(self, arm_name): # initialize the connection with the publishers and subscribers of Franka
        self.name = arm_name
        self.err_rec = rospy.Publisher('/franka_control/error_recovery/goal', ErrorRecoveryActionGoal, queue_size=10)
        rospy.sleep(1)
        self.err_rec.publish(ErrorRecoveryActionGoal())
        self.pub = rospy.Publisher('/position_joint_trajectory_controller/command', JointTrajectory, queue_size=10)
        rospy.sleep(1)
        self.sub = rospy.Subscriber('/joint_states',JointState,self.callback_read_joint_state)
        rospy.wait_for_message('/joint_states',JointState,timeout = 0.1)
        self.JS = JointState()

        self.sub_fs = rospy.Subscriber('/franka_state_controller/franka_states',FrankaState,self.callback_read_franka_state)
        rospy.wait_for_message('/franka_state_controller/franka_states',FrankaState,timeout = 0.1)
        self.FS = FrankaState()

        self.pub_gripper_grasp = rospy.Publisher('/franka_gripper/grasp/goal', GraspActionGoal, queue_size = 10)
        rospy.sleep(1)

        self.pub_gripper_open = rospy.Publisher('/franka_gripper/move/goal', MoveActionGoal, queue_size = 10)
        rospy.sleep(1)

        #radians and seconds
    def move(self, q_des, time): # define the move function in the joint space
        m = JointTrajectory()
        arm_name = self.name
        m.joint_names = [arm_name+'_joint1',
                         arm_name+'_joint2',
                         arm_name+'_joint3',
                         arm_name+'_joint4',
                         arm_name+'_joint5',
                         arm_name+'_joint6',
                         arm_name+'_joint7']
        p = JointTrajectoryPoint()
        a = q_des
        #a[0] = a[0]+angle
        p.positions = a
        p.time_from_start = rospy.Duration(time)
        p.velocities = [0]*7
        #p.accelerations = [0]*7
        p.effort = [0]*7
        m.points.append(p)

        self.pub.publish(m)

    def stop(self): # define the stop function in the joint space
        #saves joint variables values when stop was toggled
        q_stop = self.JS.position[0:7]
        global time
        #self.read_joint_state()
        m = JointTrajectory()
        arm_name = self.name
        m.joint_names = [arm_name+'_joint1',
                         arm_name+'_joint2',
                         arm_name+'_joint3',
                         arm_name+'_joint4',
                         arm_name+'_joint5',
                         arm_name+'_joint6',
                         arm_name+'_joint7']
        p = JointTrajectoryPoint()
        #p.positions = angles self.JS.position[1:7]
        a = np.asarray(self.JS.position[0:7])
        p.positions = a
        p.time_from_start = rospy.Duration(0.6)
        p.velocities = [0]*7
        #p.accelerations = [0]*7
        p.effort = [0]*7
        m.points.append(p)

        self.pub.publish(m)
        system_reaction_time = time.time()
        #timelogmain.info('stop issued time %f', system_reaction_time)
        rospy.sleep(2)

    def read_joint_state(self): # subscriber function to read the joint states
        self.sub = rospy.Subscriber('/joint_states',JointState,self.callback_read_joint_state)
        rospy.wait_for_message('/joint_states',JointState,timeout=0.1)

    def callback_read_joint_state(self,msg):
        self.JS = msg

    def callback_read_franka_state(self,msg):
        self.FS = msg

    def close_gripper(self): # function to close the gripper
        cl_gripper = GraspActionGoal()
        cl_gripper.goal.width = 0.002
        cl_gripper.goal.epsilon.inner = 0.5
        cl_gripper.goal.epsilon.outer = 0.5
        cl_gripper.goal.speed = 0.070#0.099
        cl_gripper.goal.force = 1
        self.pub_gripper_grasp.publish(cl_gripper)

    def open_gripper(self): # function to open the gripper
        open_gripper = MoveActionGoal()
        open_gripper.goal.width = 0.08
        open_gripper.goal.speed = 0.030 #1
        self.pub_gripper_open.publish(open_gripper)

def listener(): # function out of the class --> different states to control the robot motion
    #global latest_frame_time
    arm = Arm('panda')
    print("Node robot_control initialized")
    q_h = [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785]

    time_h = 7
    time_ik = 7

    ik_solver = IK("panda_link0","panda_hand") # frames for the inverse kinematics resolution

    state = 0
    move_sent = 0
    while not rospy.is_shutdown():
        rospy.sleep(0.004) #Tempo ciclo

        if True:
            if (state == 0) and (move_sent == 0): #from everywhere go home in joint space
                 arm.move(q_h,time_h)
                 arm.open_gripper()
                 move_sent = 1
                 print(arm.FS.tau_J)
            elif (state == 0) and (np.linalg.norm(np.asarray(arm.JS.position[0:7]) - q_h,2) < 0.01): # if arrived in q_h
                print("Arrived home = State 1")
                state = 1
                move_sent = 0
            elif (state == 1) and (move_sent == 0): #go to another position using the inverse kinematics
                px_pick = 0.665 # in meters
                py_pick = 0.0
                pz_pick = 0.25

                # radians
                roll  = np.pi
                pitch = 0
                yaw = 0

                quaternion_des = tf.transformations.quaternion_from_euler(roll,pitch,yaw)
                QX = quaternion_des[0]
                QY = quaternion_des[1]
                QZ = quaternion_des[2]
                QW = quaternion_des[3]

                seed_state = q_h
                q_pick = ik_solver.get_ik(seed_state,
                                          px_pick, py_pick, pz_pick, #X, Y, Z
                                          QX, QY, QZ, QW)

                arm.move(q_pick, time_ik)
                move_sent = 1
            elif (state == 1) and (np.linalg.norm(np.asarray(arm.JS.position[0:7]) - q_pick,2) < 0.01):
                print ("State 2")
                state = 2
                move_sent = 0
                print(arm.FS.tau_J)
            elif (state == 2) and (move_sent == 0): #go back home
                arm.stop()
                arm.move(q_h, time_h)
                move_sent = 1
            elif (state == 2) and (np.linalg.norm(np.asarray(arm.JS.position[0:7]) - q_h,2) < 0.01):
                print ("Arrived = home")
                state = 3
                move_sent = 0
                arm.stop()
            else:
                pass
        else:
            print("stop required")
            arm.stop()
            move_sent = 0

# define the main function that will be executed
if __name__ == '__main__':
    try:
        print("try running node")
        rospy.init_node('robot_control', anonymous = False)
        arm = Arm('panda')
        listener() # call the listener function

    except rospy.ROSInterruptException:
        pass
