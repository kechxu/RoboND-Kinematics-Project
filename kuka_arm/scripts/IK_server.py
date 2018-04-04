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


def rot_x(q):
    R_x = Matrix([[1., 0, 0],
                  [0, cos(q), -sin(q)],
                  [0, sin(q),  cos(q)]])
    return R_x


def rot_y(q):
    R_y = Matrix([[cos(q), 0, sin(q)],
                  [0, 1., 0],
                  [-sin(q), 0, cos(q)]])
    return R_y


def rot_z(q):
    R_z = Matrix([[cos(q), -sin(q), 0],
                  [sin(q), cos(q), 0],
                  [0, 0, 1.]])
    return R_z


def ht_matrix(R, t):
    ht = R.row_join(t)
    ht = ht.col_join(Matrix([[0, 0, 0, 1.]]))
    return ht


def dh_matrix(alpha, a, d, theta):
    return Matrix([[cos(theta), -sin(theta), 0, a],
        [sin(theta) * cos(alpha), cos(theta) * cos(alpha), -sin(alpha), -sin(alpha) * d],
        [sin(theta) * sin(alpha), cos(theta) * sin(alpha), cos(alpha), cos(alpha) * d],
        [0, 0, 0, 1]])


def handle_calculate_IK(req):
    rospy.loginfo("Received %s eef-poses from the plan" % len(req.poses))
    if len(req.poses) < 1:
        print "No valid poses received"
        return -1

    # Create symbols
    alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols('alpha0:7')
    a0, a1, a2, a3, a4, a5, a6 = symbols('a0:7')
    d1, d2, d3, d4, d5, d6, d7 = symbols('d1:8')
    theta1, theta2, theta3, theta4, theta5, theta6, theta7 = symbols('theta1:8')

    # Create Modified DH parameters
    s = {alpha0:     0, a0:      0, d1: 0.75
         alpha1: -pi/2, a1:   0.35, d2:    0, theta2: q2-pi/2,
         alpha2:     0, a2:   1.25, d3:    0,
         alpha3: -pi/2, a3: -0.054, d4: 1.50,
         alpha4:  pi/2, a4:      0, d5:    0,
         alpha5: -pi/2, a5:      0, d6:    0,
         alpha6:     0, a6:      0, d7: 0.303, theta7: 0}

    # Define Modified DH Transformation matrix
    T0_1 = dh_matrix(alpha0, a0, d1, theta1)
    T0_1 = T0_1.subs(s)

    T1_2 = dh_matrix(alpha1, a1, d2, theta2)
    T1_2 = T1_2.subs(s)

    T2_3 = dh_matrix(alpha2, a2, d3, theta3)
    T2_3 = T2_3.subs(s)

    T3_4 = dh_matrix(alpha3, a3, d4, theta4)
    T3_4 = T3_4.subs(s)

    T4_5 = dh_matrix(alpha4, a4, d5, theta5)
    T4_5 = T4_5.subs(s)

    T5_6 = dh_matrix(alpha5, a5, d6, theta6)
    T5_6 = T5_6.subs(s)

    T6_7 = dh_matrix(alpha6, a6, d7, theta7)
    T6_7 = T6_7.subs(s)

    # Build accumulated homogeneous transforms
    T0_2 = simplify(T0_1 * T1_2)
    T0_3 = simplify(T0_2 * T2_3)
    T0_4 = simplify(T0_3 * T3_4)
    T0_5 = simplify(T0_4 * T4_5)
    T0_6 = simplify(T0_5 * T5_6)
    T0_7 = simplify(T0_6 * T6_7)

    # Correction of gripper's rotation
    R_z = ht_matrix(rot_z(np.pi), 0)
    R_y = ht_matrix(rot_y(-np.pi/2), 0)
    R_corr = simplify(R_z * R_y)

    # Transform matrix from base link to gripper link
    T = simplify(T0_7 * R_corr)

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
    #
    #
    # Calculate joint angles using Geometric IK method
    #
    #
        ###

        # Populate response for the IK request
        # In the next line replace theta1,theta2...,theta6 by your joint angle variables
    joint_trajectory_point.positions = [joint1, joint2, joint3, joint4, joint5, joint6]
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
