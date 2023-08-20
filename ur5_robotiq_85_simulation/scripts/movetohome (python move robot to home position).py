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

group.set_named_target("arm_home")

plan1 = group.plan()

rospy.sleep(5)

group.go(wait=True)

moveit_commander.roscpp_shutdown()
