#!/usr/bin/env python

# Copyright (C) 2017 Udacity Inc.
#
# This file is part of Robotic Arm: Pick and Place project for Udacity
# Robotics nano-degree program
#
# All Rights Reserved.

# Author: Harsh Pandya

# import modules
import rospy
import tf
import math
from kuka_arm.srv import *
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Pose
from mpmath import *
from sympy import *

def matrix(alpha, a, d, q):

	ans = Matrix([[cos(q)           ,-sin(q)           ,  0         ,  a           ],
		      [sin(q)*cos(alpha), cos(q)*cos(alpha), -sin(alpha), -sin(alpha)*d],
		      [sin(q)*sin(alpha), cos(q)*sin(alpha),  cos(alpha),  cos(alpha)*d],
		      [0                , 0                ,  0         ,  1           ]])

	return ans


def rot_x(angle):
	R_x = Matrix([[   1,                0,               0,     0],
                       [     0,       cos(angle),     -sin(angle),     0],
                       [     0,       sin(angle),      cos(angle),     0],
                       [     0,                0,               0,     1]])
        return R_x

def rot_y(angle):
        R_y = Matrix([[   cos(angle),                0,    sin(angle),     0],
                        [             0,                1,             0,     0],
                        [   -sin(angle),                0,    cos(angle),     0],
                        [             0,                0,             0,     1]])
        return R_y

def rot_z(angle):
       R_z = Matrix([[      cos(angle),      -sin(angle),          0,      0],
                       [       sin(angle),       cos(angle),          0,      0],
                       [                0,                0,          1,      0],
                       [                0,                0,          0,      1]])
       return R_z


def handle_calculate_IK(req):
    rospy.loginfo("Received %s eef-poses from the plan" % len(req.poses))
    if len(req.poses) < 1:
        print "No valid poses received"
        return -1
    else:

        ### Your FK code here
        # Create symbols
	q1, q2, q3, q4, q5, q6, q7 = symbols('q1:8') #theta
        d1, d2, d3, d4, d5, d6, d7 = symbols('d1:8')
        a0, a1, a2, a3, a4, a5, a6 = symbols('a0:7')
        alpha0, alpha1, alpha2, alpha3, alpha4, alpha5 , alpha6 = symbols('alpha0:7')

	# Create Modified DH parameters

	s = {alpha0:     0,  a0:     0,  d1:   0.75,
             alpha1: -pi/2,  a1:  0.35,  d2:      0, q2:  q2-pi/2,
             alpha2:     0,  a2:  1.25,  d3:      0,
             alpha3: -pi/2,  a3:-0.054,  d4:    1.5,
             alpha4:  pi/2,  a4:     0,  d5:      0,
             alpha5: -pi/2,  a5:     0,  d6:      0,
             alpha6:     0,  a6:     0,  d7:  0.303, q7:     0}

	# Define Modified DH Transformation matrix

	# Create individual transformation matrices
	T0_1 = matrix(alpha=alpha0,a= a0, d=d1,q=q1)
	T0_1 = T0_1.subs(s)

	T1_2 = matrix(alpha=alpha1, a=a1, d=d2,q=q2)
        T1_2 = T1_2.subs(s)

	T2_3 = matrix(alpha=alpha2, a=a2, d=d3,q=q3)
        T2_3 = T2_3.subs(s)

	#T3_4 = matrix(alpha3, a3, d4,q4)
        #T3_4.subs(s)

	#T4_5 = matrix(alpha4, a4, d5,q5)
        #T4_5.subs(s)

	#T5_6 = matrix(alpha5, a5, d6,q6)
        #T5_6.subs(s)

	#T6_G = matrix(alpha6, a6, d7,q7)
        #T6_G.subs(s)

	#T0_G = simplify(T0_1 * T1_2 * T2_3 * T3_4 * T4_5 * T5_6 * T6_G)

	R_corr = simplify(rot_z(pi) * rot_y(-pi/2))

        # Initialize service response
        joint_trajectory_list = []
        for x in xrange(0, len(req.poses)):
            # IK code starts here
            joint_trajectory_point = JointTrajectoryPoint()

	    # Extract end-effector position and orientation from request
	    # px,py,pz = end-effector position
	    # roll, pitch, yaw = end-effector orientation
            px = req.poses[x].position.x
            py = req.poses[x].position.y
            pz = req.poses[x].position.z

            (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
                [req.poses[x].orientation.x, req.poses[x].orientation.y,
                    req.poses[x].orientation.z, req.poses[x].orientation.w])

            ### Your IK code here
	    # Compensate for rotation discrepancy between DH parameters and Gazebo
	    # End-factor rotation matrix
	    R_EE = rot_z(yaw)[0:3,0:3] * rot_y(pitch)[0:3,0:3] * rot_x(roll)[0:3,0:3] * R_corr[0:3,0:3]
	    EE_Position = Matrix([[px],[py],[pz]])
	    WC = EE_Position - 0.303 * R_EE[:,2] # EE is just transmition along z axis relative to WC
	    # Calculate joint angles using Geometric IK method
	    theta_1 = atan2(WC[1],WC[0])

	    # cal theta_2 we need to get lengh of B and angle of a (in lesson figure)
	    new_wx = sqrt(WC[0]**2 + WC[1]**2) - 0.35
	    new_wz = WC[2] - 0.75 # WC_Z - d1
	    B = sqrt(new_wx**2 + new_wz**2)

	    # A and C fixed length from urdf
	    C = 1.25
            A = 1.5

	    # cos(a) = (B^2 + C^2 - A^2) / (2* C * A)
	    angle_a = math.acos(( pow(B,2) + pow(C,2) - pow(A,2) ) / ( 2 * B * C ))
	    theta_2 = pi/2 - angle_a - atan2(new_wz,new_wx)

	    # to get theta 3 we have to calculate angle_b first as follows:-
            angle_b = math.acos((pow(C,2) + pow(A,2) - pow(B,2)) / (2 * C * A))
            theta_3 = pi/2 - angle_b - 0.03598 # 0.03598 is fixed angle = atan2(0.054,1.5)

	    # get theta 3,4,5

            T0_2 = simplify(T0_1 * T1_2)
            T0_3 = simplify(T0_2 * T2_3)

            R0_3 = T0_3.evalf(subs={q1: theta_1, q2: theta_2, q3: theta_3})[0:3,0:3]
	    #print(R0_3)
	    #print(R_EE)
	    R3_6 = R0_3.inv("LU") * R_EE
  	    theta_4 = atan2(R3_6[2, 2], -R3_6[0, 2])
            theta_5 = atan2(sqrt(R3_6[0, 2]*R3_6[0, 2]+R3_6[2, 2]*R3_6[2, 2]), R3_6[1, 2])
            theta_6 = atan2(-R3_6[1, 1], R3_6[1, 0])
            # Populate response for the IK request
            # In the next line replace theta1,theta2...,theta6 by your joint angle variables
	    joint_trajectory_point.positions = [theta_1, theta_2, theta_3, theta_4, theta_5, theta_6]
	    joint_trajectory_list.append(joint_trajectory_point)

        rospy.loginfo("length of Joint Trajectory List: %s" % len(joint_trajectory_list))
        return CalculateIKResponse(joint_trajectory_list)


def IK_server():
    # initialize node and declare calculate_ik service
    rospy.init_node('IK_server')
    s = rospy.Service('calculate_ik', CalculateIK, handle_calculate_IK)
    print "Ready to receive an IK request"
    rospy.spin()

if __name__ == "__main__":
    IK_server()
