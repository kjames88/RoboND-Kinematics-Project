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
import numpy as np

# some values out of matrix inverse are very slightly out of range for sin/cos (e.g. 1.00001172958) leading to nan
def limit(sin_cos_val):
    if sin_cos_val < -1.0:
        return -1.0
    elif sin_cos_val > 1.0:
        return 1.0
    return sin_cos_val

def normalize(rad):
    while rad > np.pi:
        rad -= (2.*np.pi)
    while rad < -np.pi:
        rad += (2.*np.pi)
    return rad

def handle_calculate_IK(req):
    rospy.loginfo("Received %s eef-poses from the plan" % len(req.poses))
    if len(req.poses) < 1:
        print "No valid poses received"
        return -1
    else:
        print "Run IK on poses!"

        ### Your FK code here
        # Create symbols
	#
	#

        q1,q2,q3,q4,q5,q6,q7 = symbols('q1:8')
        d1,d2,d3,d4,d5,d6,d7 = symbols('d1:8')
        a0,a1,a2,a3,a4,a5,a6 = symbols('a0:7')
        alpha0,alpha1,alpha2,alpha3,alpha4,alpha5,alpha6 = symbols('alpha0:7')

	# Create Modified DH parameters
	#
	#

        # DH parameters
        #   symbol dictionary
        s = {alpha0:     0, a0:      0, d1:  0.75,
             alpha1: -pi/2, a1:   0.35, d2:     0, q2: q2-pi/2,
             alpha2:     0, a2:   1.25, d3:     0,
             alpha3: -pi/2, a3: -0.054, d4:  1.50,
             alpha4:  pi/2, a4:      0, d5:     0,
             alpha5: -pi/2, a5:      0, d6:     0,
             alpha6:     0, a6:      0, d7: 0.303, q7: 0}

	# Define Modified DH Transformation matrix
	#
	#

        T0_1 = Matrix([[cos(q1), -sin(q1), 0, a0],
                       [sin(q1) * cos(alpha0), cos(q1) * cos(alpha0), -sin(alpha0), -sin(alpha0) * d1],
                       [sin(q1) * sin(alpha0), cos(q1) * sin(alpha0), cos(alpha0), cos(alpha0) * d1],
                       [0, 0, 0, 1]])
        T0_1 = T0_1.subs(s)

        T1_2 = Matrix([[cos(q2), -sin(q2), 0, a1],
                       [sin(q2) * cos(alpha1), cos(q2) * cos(alpha1), -sin(alpha1), -sin(alpha1) * d2],
                       [sin(q2) * sin(alpha1), cos(q2) * sin(alpha1), cos(alpha1), cos(alpha1) * d2],
                       [0, 0, 0, 1]])
        T1_2 = T1_2.subs(s)

        T2_3 = Matrix([[cos(q3), -sin(q3), 0, a2],
                       [sin(q3) * cos(alpha2), cos(q3) * cos(alpha2), -sin(alpha2), -sin(alpha2) * d3],
                       [sin(q3) * sin(alpha2), cos(q3) * sin(alpha2), cos(alpha2), cos(alpha2) * d3],
                       [0, 0, 0, 1]])
        T2_3 = T2_3.subs(s)

        T3_4 = Matrix([[cos(q4), -sin(q4), 0, a3],
                       [sin(q4) * cos(alpha3), cos(q4) * cos(alpha3), -sin(alpha3), -sin(alpha3) * d4],
                       [sin(q4) * sin(alpha3), cos(q4) * sin(alpha3), cos(alpha3), cos(alpha3) * d4],
                       [0, 0, 0, 1]])
        T3_4 = T3_4.subs(s)

        T4_5 = Matrix([[cos(q5), -sin(q5), 0, a4],
                       [sin(q5) * cos(alpha4), cos(q5) * cos(alpha4), -sin(alpha4), -sin(alpha4) * d5],
                       [sin(q5) * sin(alpha4), cos(q5) * sin(alpha4), cos(alpha4), cos(alpha4) * d5],
                       [0, 0, 0, 1]])
        T4_5 = T4_5.subs(s)

        T5_6 = Matrix([[cos(q6), -sin(q6), 0, a5],
                       [sin(q6) * cos(alpha5), cos(q6) * cos(alpha5), -sin(alpha5), -sin(alpha5) * d6],
                       [sin(q6) * sin(alpha5), cos(q6) * sin(alpha5), cos(alpha5), cos(alpha5) * d6],
                       [0, 0, 0, 1]])
        T5_6 = T5_6.subs(s)

        T6_G = Matrix([[cos(q7), -sin(q7), 0, a6],
                       [sin(q7) * cos(alpha6), cos(q7) * cos(alpha6), -sin(alpha6), -sin(alpha6) * d7],
                       [sin(q7) * sin(alpha6), cos(q7) * sin(alpha6), cos(alpha6), cos(alpha6) * d7],
                       [0, 0, 0, 1]])
        T6_G = T6_G.subs(s)
        # rotate 180 degrees about Z and -90 degrees about Y
        Rz = Matrix([[cos(pi), -sin(pi), 0, 0],
                     [sin(pi), cos(pi), 0, 0],
                     [0, 0, 1, 0],
                     [0, 0, 0, 1]])

        Ry = Matrix([[cos(-pi/2), 0, sin(-pi/2), 0],
                     [0, 1, 0, 0],
                     [-sin(-pi/2), 0, cos(-pi/2), 0],
                     [0, 0, 0, 1]])
        Rcorr = (Rz * Ry)
        T0_2 = (T0_1 * T1_2)  # base to link 2
        T0_3 = (T0_2 * T2_3)  # base to link 3
        T0_4 = (T0_3 * T3_4)  # base to link 4
        T0_5 = (T0_4 * T4_5)  # base to link 5
        T0_6 = (T0_5 * T5_6)  # base to link 6
        T0_G = (T0_6 * T6_G)  # base to gripper before urdf/dh adjustment

        T_all = (T0_G * Rcorr)


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
        R0_3 = R0_1*R1_2*R2_3
        R3_6_sym = R3_4*R4_5*R5_6

        #print('R3_6_sym = {}'.format(R3_6_sym))

        lv = 0.303
        d6v = 0.0

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

            px_intent = px
            py_intent = py
            pz_intent = pz

            (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
                [req.poses[x].orientation.x, req.poses[x].orientation.y,
                    req.poses[x].orientation.z, req.poses[x].orientation.w])


            r,p,y = symbols('r p y')
            Rx_roll = Matrix([[1.,0.,0.],
                              [0.,cos(r),-sin(r)],
                              [0.,sin(r),cos(r)]])

            Ry_pitch = Matrix([[cos(p),0.,sin(p)],
                              [0.,1.,0.],
                              [-sin(p),0.,cos(p)]])

            Rz_yaw = Matrix([[cos(y),-sin(y),0.],
                             [sin(y),cos(y),0.],
                             [0.,0.,1.]])

            Rz = Matrix([[cos(pi), -sin(pi), 0.],
                         [sin(pi), cos(pi), 0.],
                         [0., 0., 1.]])
            Ry = Matrix([[cos(-pi/2.), 0., sin(-pi/2.)],
                         [0., 1., 0.],
                         [-sin(-pi/2.), 0., cos(-pi/2.)]])
            R_corr = Rz*Ry
            R_ee = Rz_yaw*Ry_pitch*Rx_roll
            R_ee = R_ee*R_corr
            Rrpy = R_ee.subs({'r':roll,'p':pitch,'y':yaw})

            ### Your IK code here

            nx = float(Rrpy[0,2])
            ny = float(Rrpy[1,2])
            nz = float(Rrpy[2,2])
            wx = px - ((d6v + lv) * nx)
            wy = py - ((d6v + lv) * ny)
            wz = pz - ((d6v + lv) * nz)

            #print('nx {} ny {} nz {} wx {} wy {} wz {}'.format(nx,ny,nz,wx,wy,wz))
            theta1v = np.arctan2(wy,wx) #numeric
            theta1v = normalize(theta1v)

            #print('theta1v = {}'.format(theta1v))

            j2x = s[a1] * np.cos(theta1v) #numeric
            j2y = s[a1] * np.sin(theta1v) #numeric
            j2z = s[d1]

            dx = wx - j2x #numeric
            dy = wy - j2y #numeric
            dz = wz - j2z #numeric

            # Compute x/y plane component by subtracting a1
            xy_mag = np.sqrt(wx**2 + wy**2) - s[a1]

	    # Calculate joint angles using Geometric IK method
	    #
	    #
            ###
            A = np.sqrt(s[a3]**2 + s[d4]**2)
            B = np.sqrt((np.sqrt(wx**2 + wy**2) - s[a1])**2 + dz**2)
            #B = np.sqrt(pow(xy_mag,2) + pow(wz-s[d1],2))
            C = s[a2]
            v = (A**2 + C**2 - B**2) / (2.0*A*C)
            b = np.arccos(v)
            # angle adjust component due to a3 (sag in link 4 from project walkthrough)
            a_sag = np.arctan2(s[a3], s[d4])
            theta3v = (np.pi/2. - b) + a_sag
            theta3v = normalize(theta3v)

            #print('theta3v = {}'.format(theta3v))

            v = (B**2 + C**2 - A**2) / (2.0*B*C)
            a = np.arccos(v)

            dxy = np.sqrt(dx**2 + dy**2)
            angle = np.arctan2(dz,dxy)
            theta2v = np.pi/2. - angle - a
            theta2v = normalize(theta2v)
            #print('angle to wc {} theta2v = {}'.format(angle,theta2v))

            R0_3 = T0_1[0:3,0:3]*T1_2[0:3,0:3]*T2_3[0:3,0:3]
            R0_3 = R0_3.evalf(subs={q1:theta1v,q2:theta2v,q3:theta3v})
            #R3_6 = R0_3.inv('LU')*Rrpy
            R3_6 = R0_3.inv()*Rrpy  # LU causes significant error

            # per the project walkthrough, use the arctan2 method rather than arccos and arcsin
            theta5s = atan2(sqrt(R3_6[0,2]*R3_6[0,2] + R3_6[2,2]*R3_6[2,2]),R3_6[1,2])
            theta5v = float(theta5s.evalf())
            theta5v = normalize(theta5v)
            # keep the quadrant for theta4,theta6 when sin(theta5) changes from positive to negative
            if np.sin(theta5v) < 0:
                theta4s = atan2(-R3_6[2,2],R3_6[0,2])
                theta6s = atan2(R3_6[1,1],-R3_6[1,0])
            else:
                theta4s = atan2(R3_6[2,2],-R3_6[0,2])
                theta6s = atan2(-R3_6[1,1],R3_6[1,0])
            theta4v = float(theta4s.evalf())
            theta6v = float(theta6s.evalf())
            theta4v = normalize(theta4v)
            theta6v = normalize(theta6v)

            rospy.loginfo('theta1 {} theta2 {} theta3 {} theta4 {} theta5 {} theta6 {}'. \
                    format(theta1v,theta2v,theta3v,theta4v,theta5v,theta6v))

            ts = {q1: theta1v, q2: theta2v, q3: theta3v, q4: theta4v, q5: theta5v, q6: theta6v, q7: 0.}
            px = T_all[0,3].evalf(subs=ts)
            py = T_all[1,3].evalf(subs=ts)
            pz = T_all[2,3].evalf(subs=ts)

            rospy.loginfo('intent {} {} {} FK {} {} {}'.format(px_intent,py_intent,pz_intent,px,py,pz))
            rospy.loginfo('intent quaternion {} {} {} {}'.format(req.poses[x].orientation.x, req.poses[x].orientation.y, \
                          req.poses[x].orientation.z, req.poses[x].orientation.w))
            rospy.loginfo('IK/FK disparity {} {} {}'.format(abs(px-px_intent),abs(py-py_intent),abs(pz-pz_intent)))

            # Populate response for the IK request
            # In the next line replace theta1,theta2...,theta6 by your joint angle variables
	    joint_trajectory_point.positions = [theta1v, theta2v, theta3v, theta4v, theta5v, theta6v]
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
