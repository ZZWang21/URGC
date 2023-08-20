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

        # Set the reference frame for pose targets
        reference_frame = "base_link"

        # Set the ur5_arm reference frame accordingly
        self.arm.set_pose_reference_frame(reference_frame)

        # Allow replanning to increase the odds of a solution
        self.arm.allow_replanning(True)

        # Allow some leeway in position (meters) and orientation (radians)
        self.arm.set_goal_position_tolerance(0.1)  #ORI:0.01
        self.arm.set_goal_orientation_tolerance(0.1)

        # Get the current pose so we can add it as a waypoint
        start_pose = self.arm.get_current_pose(end_effector_link).pose

        # Initialize the waypoints list
        waypoints = []

        # Set the first waypoint to be the starting pose
        # Append the pose to the waypoints list
        waypoints.append(start_pose)

        

        # use FOR cycle to define 12 points of waypoints. 3 sequences:
        #classes = ("geometry_msgs.msg._Pose.Pose")
        #as_list = list(classes)
        wpose0 = self.arm.get_current_pose(end_effector_link).pose
        rospy.loginfo(str(type(wpose0)) )
        Npoint=12
        wpose=[None]*(Npoint+1)
        for ii in range(1,Npoint):
            wpose[0]=[]
            wpose[0] = deepcopy(wpose0)
            #1:
            wpose[ii]=[]             
            wpose[ii]= deepcopy(wpose[(ii-1)])
            if ii%4==1:
                wpose[ii].position.x = wpose[(ii-1)].position.x-0.1
                waypoints.append(deepcopy(wpose[ii]))
                rospy.loginfo("WAYPOSE " + str(ii) +":" + str(wpose[ii].position) )
            
            #2:
            if ii%2==0:
                rospy.loginfo(ii)
                rospy.loginfo(wpose[ii-1])
                wpose[ii]=[]             
                wpose[ii]= deepcopy(wpose[ii-1])
                #wpose[(ii-1)*4+2] = deepcopy(wpose[(ii-1)*4+1])
                wpose[ii].position.y = wpose[(ii-1)].position.y + 0.05
                waypoints.append(deepcopy(wpose[ii]))
            #3:
            if ii%4==3:
                wpose[ii]=[]             
                wpose[ii]= deepcopy(wpose[(ii-1)])
                #wpose[(ii-1)*4+3] = deepcopy(wpose[(ii-1)*4+2])
                wpose[ii].position.x = wpose[(ii-1)].position.x + 0.1
                waypoints.append(deepcopy(wpose[ii]))
                rospy.loginfo("WAYPOSE " + str(ii) +":" + str(wpose[ii].position) )

        fraction = 0.001
        maxtries = 100
        attempts = 0

        # Set the internal state to the current state
        self.arm.set_start_state_to_current_state()

        # Plan the Cartesian path connecting the waypoints
        while fraction < 0.8 and attempts < maxtries:
            (plan, fraction) = self.arm.compute_cartesian_path (waypoints, 0.001, 0.0, True)

            # Increment the number of attempts
            attempts += 1

            # Print out a progress message
            if attempts % 10 == 0:
                rospy.loginfo("Still trying after " + str(attempts) + " attempts...")
                rospy.loginfo("FRACTION: " + str(fraction) )

        # If we have a complete plan, execute the trajectory
        if fraction >=0.8: #== 1.0:
            rospy.loginfo("Path computed successfully. Moving the arm.")
            num_pts = len(plan.joint_trajectory.points)

            rospy.loginfo("\n# waypoints: "+str(num_pts))
            waypoints = []
            for i in range(num_pts):
                waypoints.append(plan.joint_trajectory.points[i].positions)

            #rospy.init_node('send_joints')
            pub = rospy.Publisher('/arm_controller/command',
                                  JointTrajectory,
                                  queue_size=10)

            # Create the topic message
            traj = JointTrajectory()
            traj.header = Header()
            # Joint names for UR5
            traj.joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint',
                                'elbow_joint', 'wrist_1_joint', 'wrist_2_joint',
                                'wrist_3_joint']

            rate = rospy.Rate(20)
            cnt = 0
            pts = JointTrajectoryPoint()
            #this should be the cycle:
            while not rospy.is_shutdown():

                cnt += 1
                traj.header.stamp = rospy.Time.now()

                #        pts.positions = [0.0, -2.33, 1.57, 0.0, 0.0, 0.0]

                pts.positions = waypoints[cnt % num_pts]

                #rospy.loginfo("CNT:"+ str(cnt))
                pts.time_from_start = rospy.Duration(0.001*cnt)
                #rospy.loginfo("TIME:"+ str(pts.time_from_start))
                # Set the points to the trajectory
                traj.points = []
                traj.points.append(pts)
                # Publish the message
                pub.publish(traj)

                rate.sleep()

        # self.arm.execute(plan)
            rospy.loginfo("Path execution complete.")
        else:
            rospy.loginfo("Path planning failed with only " + str(fraction) + " success after " + str(maxtries) + " attempts.")

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