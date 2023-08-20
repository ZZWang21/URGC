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
#import forcezz


from trajectory_msgs.msg import JointTrajectory

from trajectory_msgs.msg import JointTrajectoryPoint

import numpy as np

rospy.init_node("moveit_cartesian_path", anonymous=False)

robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
group = moveit_commander.MoveGroupCommander("arm")

# Set the reference frame and end effector link
group.set_pose_reference_frame("base_link")
group.set_end_effector_link("tool0") #tool0

# Set the target pose
target_pose = Pose()
target_pose.position.x = 0  # Horizontal position (relative to the base)
target_pose.position.y = 0.3  # Horizontal position (relative to the base)
target_pose.position.z = 0.2  # Desired vertical position (relative to the base)

# Set the orientation (e.g., keep it aligned with the world frame)
target_pose.orientation.x = 0.0
target_pose.orientation.y = 1.5
target_pose.orientation.z = -0.707
target_pose.orientation.w = 0.7

# Plan and execute the motion
group.set_pose_target(target_pose)
#plan = group.plan()

# Execute the motion
#group.execute(plan, wait=True)
group.go(wait=True)

# Clear targets
group.clear_pose_targets()

# Stop the ROS node
rospy.signal_shutdown('End effector movement complete')