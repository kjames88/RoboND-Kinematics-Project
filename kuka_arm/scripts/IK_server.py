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
from kuka_arm.srv import *
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Pose
from mpmath import *
from sympy import *


def handle_calculate_IK(req):
    rospy.loginfo("Received %s eef-poses from the plan" % len(req.poses))
    if len(req.poses) < 1:
        print "No valid poses received"
        return -1
    else:

        ### Your FK code here
        # Create symbols
	#
	#

        theta1,theta2,theta3,theta4,theta5,theta6,theta7 = symbols('theta1:8')
        d1,d2,d3,d4,d5,d6,d7 = symbols('d1:8')
        a0,a1,a2,a3,a4,a5,a6 = symbols('a0:7')
        alpha0,alpha1,alpha2,alpha3,alpha4,alpha5,alpha6 = symbols('alpha0:7')

	# Create Modified DH parameters
	#
	#

        # DH parameters
        #   symbol dictionary
        s = {alpha0:     0, a0:      0, d1:  0.75,
             alpha1: -pi/2, a1:   0.35, d2:     0, theta2: theta2-pi/2,
             alpha2:     0, a2:   1.25, d3:     0,
             alpha3: -pi/2, a3: -0.054, d4:  1.50,
             alpha4:  pi/2, a4:      0, d5:     0,
             alpha5: -pi/2, a5:      0, d6:     0,
             alpha6:     0, a6:      0, d7: 0.303, theta7: 0}

	# Define Modified DH Transformation matrix
	#
	#

        T0_1 = Matrix([[cos(theta1), -sin(theta1), 0, a0],
                       [sin(theta1) * cos(alpha0), cos(theta1) * cos(alpha0), -sin(alpha0), -sin(alpha0) * d1],
                       [sin(theta1) * sin(alpha0), cos(theta1) * sin(alpha0), cos(alpha0), cos(alpha0) * d1],
                       [0, 0, 0, 1]])
        T0_1 = T0_1.subs(s)

        T1_2 = Matrix([[cos(theta2), -sin(theta2), 0, a1],
                       [sin(theta2) * cos(alpha1), cos(theta2) * cos(alpha1), -sin(alpha1), -sin(alpha1) * d2],
                       [sin(theta2) * sin(alpha1), cos(theta2) * sin(alpha1), cos(alpha1), cos(alpha1) * d2],
                       [0, 0, 0, 1]])
        T1_2 = T1_2.subs(s)

        T2_3 = Matrix([[cos(theta3), -sin(theta3), 0, a2],
                       [sin(theta3) * cos(alpha2), cos(theta3) * cos(alpha2), -sin(alpha2), -sin(alpha2) * d3],
                       [sin(theta3) * sin(alpha2), cos(theta3) * sin(alpha2), cos(alpha2), cos(alpha2) * d3],
                       [0, 0, 0, 1]])
        T2_3 = T2_3.subs(s)

        T3_4 = Matrix([[cos(theta4), -sin(theta4), 0, a3],
                       [sin(theta4) * cos(alpha3), cos(theta4) * cos(alpha3), -sin(alpha3), -sin(alpha3) * d4],
                       [sin(theta4) * sin(alpha3), cos(theta4) * sin(alpha3), cos(alpha3), cos(alpha3) * d4],
                       [0, 0, 0, 1]])
        T3_4 = T3_4.subs(s)

        T4_5 = Matrix([[cos(theta5), -sin(theta5), 0, a4],
                       [sin(theta5) * cos(alpha4), cos(theta5) * cos(alpha4), -sin(alpha4), -sin(alpha4) * d5],
                       [sin(theta5) * sin(alpha4), cos(theta5) * sin(alpha4), cos(alpha4), cos(alpha4) * d5],
                       [0, 0, 0, 1]])
        T4_5 = T4_5.subs(s)

        T5_6 = Matrix([[cos(theta6), -sin(theta6), 0, a5],
                       [sin(theta6) * cos(alpha5), cos(theta6) * cos(alpha5), -sin(alpha5), -sin(alpha5) * d6],
                       [sin(theta6) * sin(alpha5), cos(theta6) * sin(alpha5), cos(alpha5), cos(alpha5) * d6],
                       [0, 0, 0, 1]])
        T5_6 = T5_6.subs(s)

        T6_G = Matrix([[cos(theta7), -sin(theta7), 0, a6],
                       [sin(theta7) * cos(alpha6), cos(theta7) * cos(alpha6), -sin(alpha6), -sin(alpha6) * d7],
                       [sin(theta7) * sin(alpha6), cos(theta7) * sin(alpha6), cos(alpha6), cos(alpha6) * d7],
                       [0, 0, 0, 1]])
        T6_G = T6_G.subs(s)


	# Create individual transformation matrices
	#
	#



	# Extract rotation matrices from the transformation matrices
	#
	#
        ###

        R0_1 = T0_1[0:3,0:3]
        R1_2 = T1_2[0:3,0:3]
        R2_3 = T2_3[0:3,0:3]
        R3_4 = T3_4[0:3,0:3]
        R4_5 = T4_5[0:3,0:3]
        R5_6 = T5_6[0:3,0:3]

        l = 0.303
        d6 = 0

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

            Rx_roll = Matrix([[1,0,0],
                              [0,cos(roll),-sin(roll)],
                              [0,sin(roll),cos(roll)]])

            Ry_pitch = Matrix([[cos(pitch),0,sin(pitch)],
                               [0,1,0],
                               [-sin(pitch),0,cos(pitch)]])

            Rz_yaw = Matrix([[cos(yaw),-sin(yaw),0],
                             [sin(yaw),cos(yaw),0],
                             [0,0,1]])

	    # Compensate for rotation discrepancy between DH parameters and Gazebo
	    #
	    #
            Rz = Matrix([[cos(pi), -sin(pi), 0],
                         [sin(pi), cos(pi), 0],
                         [0, 0, 1]])

            Ry = Matrix([[cos(-pi/2), 0, sin(-pi/2)],
                         [0, 1, 0],
                         [-sin(-pi/2), 0, cos(-pi/2)]])
            R_corr = (Rz * Ry)
            Rrpy = Rz_yaw * Ry_pitch * Rx_roll * R_corr

            ### Your IK code here

            nx = Rrpy[0,2]
            ny = Rrpy[1,2]
            nz = Rrpy[2,2]
            wx = px - ((d6 + l) * nx)
            wy = py - ((d6 + l) * ny)
            wz = pz - ((d6 + l) * nz)

            theta1 = atan2(wy,wx)
            j2x = s[a1] * cos(theta1).evalf()
            j2y = s[a1] * sin(theta1).evalf()
            j2z = s[d1]

            dx = wx - j2x
            dy = wy - j2y
            dz = wz - j2z

	    # Calculate joint angles using Geometric IK method
	    #
	    #
            ###
            A = sqrt(s[a3]**2 + s[d4]**2)
            B = sqrt(dx**2 + dy**2 + dz**2)
            C = s[a2]
            v = (A**2 + C**2 - B**2) / (2.0*A*C)
            b = acos(v)
            theta3 = (pi/2 - b)

            R0_3 = R0_1*R1_2*R2_3
            R0_3 = R0_3.evalf()
            R3_6 = R3_4*R4_5*R5_6
            R3_6 = R0_3.inv('LU')*Rrpy


            # Populate response for the IK request
            # In the next line replace theta1,theta2...,theta6 by your joint angle variables
	    joint_trajectory_point.positions = [theta1, theta2, theta3, theta4, theta5, theta6]
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
