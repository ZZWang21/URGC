#!/usr/bin/env python

"""
    moveit_cartesian_path.py - Version 0.1 2016-07-28
    Based on the R. Patrick Goebel's moveit_cartesian_demo.py demo code.
    Plan and execute a Cartesian path for the end-effector.
    Created for the Pi Robot Project: http://www.pirobot.org
    Copyright (c) 2014 Patrick Goebel.  All rights reserved.
    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.5
    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details at:
    http://www.gnu.org/licenses/gpl.html
"""

import rospy, sys
import rospkg
import moveit_commander
from geometry_msgs.msg import Pose
from copy import deepcopy
from std_msgs.msg import Header
from std_msgs.msg import String
from std_msgs.msg import Float64
from tf_conversions import posemath
from PyKDL import *
import tf
import math
#import forcezz

from trajectory_msgs.msg import JointTrajectory

from trajectory_msgs.msg import JointTrajectoryPoint
from geometry_msgs.msg import PoseStamped, Quaternion
import tf

import numpy as np

class MoveItCartesianPath:
    def __init__(self):
        rospy.init_node("moveit_cartesian_path", anonymous=False)

        rospy.loginfo("Starting node moveit_cartesian_path")

        rospy.on_shutdown(self.cleanup)

        # Initialize the move_group API
        moveit_commander.roscpp_initialize(sys.argv)

        # Initialize the move group for the ur5_arm
        self.arm = moveit_commander.MoveGroupCommander('arm')

        # Get the name of the end-effector link
        end_effector_link = self.arm.get_end_effector_link()
        print("ZZW END:",end_effector_link)

        # Set the reference frame for pose targets
        reference_frame = "base_link"

        # Set the ur5_arm reference frame accordingly
        self.arm.set_pose_reference_frame(reference_frame)

        # Allow replanning to increase the odds of a solution
        self.arm.allow_replanning(True)

        # Allow some leeway in position (meters) and orientation (radians)
        self.arm.set_goal_position_tolerance(0.01)  #ORI:0.01
        self.arm.set_goal_orientation_tolerance(0.01)

        # Get the current pose so we can add it as a waypoint
        start_pose = self.arm.get_current_pose(end_effector_link).pose

        # Initialize the waypoints list
        waypoints = []

        # Set the first waypoint to be the starting pose
        # Append the pose to the waypoints list
        waypoints.append(start_pose)
        print(waypoints)

        waypoints1 = start_pose
        #waypoints1.position.x = 0
        #waypoints1.position.y = 0.2
        #waypoints1.position.z = -0.05
        # Define the target position (x, y, z) and the normal vector to the surface (nx, ny, nz)
        target_position = [0.4, 0.0, 0.2]
        surface_normal = [0, 0, 1]

        #yaw = math.pi/2.0  # arbitrary yaw angle (in radians)
        #quaternion = tf.transformations.quaternion_from_euler(0, 0, yaw, 'rzyx')
        quat = np.array([0.0, 0.0, 0.0, 1.0])
        theta = math.pi
        axis = np.array([1.0, 0.0, 0.0])
        rot = tf.transformations.quaternion_about_axis(theta, axis)
        new_quat = tf.transformations.quaternion_multiply(quat, rot)
        print("NEW_QUAT:",new_quat)

        # Compute the quaternion corresponding to a rotation of 90 degrees around the X-axis
        #q = tf.transformations.quaternion_from_euler(0, 0, np.pi/4)

        # Create a PoseStamped message with the target position and orientation
        target_pose = PoseStamped()
        target_pose.header.frame_id = 'base_link'
        target_pose.pose.position.x = target_position[0]
        target_pose.pose.position.y = target_position[1]
        target_pose.pose.position.z = target_position[2]
        target_pose.pose.orientation.x = new_quat[0]#0#quaternion[0]#-0.30916991834419455
        target_pose.pose.orientation.y = new_quat[1]#0#quaternion[1]#0.9434560992255728
        target_pose.pose.orientation.z = new_quat[2]#0#quaternion[2]#0.0646617033253340
        target_pose.pose.orientation.w = new_quat[3]#-1#quaternion[3]#0.10061518050561072
        waypoints.append(target_pose)
        #(plan, fraction) = self.arm.compute_cartesian_path (waypoints, 0.001, 0.0, True)
        self.arm.set_pose_target(target_pose)#, end_effector_link)
        #self.arm.set_pose_target(target_pose)

        # Plan and execute the trajectory to move the end effector to the target pose
        plan = self.arm.go(wait=True)
        self.arm.stop()
        self.arm.clear_pose_targets()

    def cleanup(self):
        rospy.loginfo("Stopping the robot")

        # Stop any current arm movement
        self.arm.stop()

        #Shut down MoveIt! cleanly
        rospy.loginfo("Shutting down Moveit!")
        moveit_commander.roscpp_shutdown()
        moveit_commander.os._exit(0)
# Force measurement 20220108:::: try class?


if __name__ == "__main__":
    try:
        MoveItCartesianPath()
    except KeyboardInterrupt:
        print ("Shutting down MoveItCartesianPath node.")
