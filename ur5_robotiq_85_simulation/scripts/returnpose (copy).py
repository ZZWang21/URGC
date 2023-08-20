#!/usr/bin/python3
import rospy
from std_srvs.srv import Empty, EmptyRequest  #ZZW 20221229
from gazebo_msgs.srv import GetModelState, SetModelConfiguration, DeleteModel, \
    SpawnModel
from geometry_msgs.msg import Pose
from controller_manager_msgs.srv import SwitchController, SwitchControllerRequest
from moveit_msgs.msg import PlanningScene, PlanningSceneComponents
from moveit_msgs.srv import GetPlanningScene
import moveit_commander #import MoveGroupCommander
from actionlib import SimpleActionClient
from control_msgs.msg import FollowJointTrajectoryAction, \
    FollowJointTrajectoryGoal
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint
from tf.transformations import quaternion_from_euler
from math import pi
from copy import deepcopy
import time
from tf_conversions import posemath, toMsg
import PyKDL
from threading import Timer

class SmartUR5e(object):

    __current_model_name = "cricket_ball"
    __get_pose_srv = rospy.ServiceProxy("/gazebo/get_model_state", GetModelState)

    def __init__(self):
        self.__get_pose_srv = rospy.ServiceProxy("/gazebo/get_model_state", GetModelState)

        ball_pose = self.get_object_pose()

        print("wowow")#self.get_object_pose)

        

    def get_object_pose(self):
        """
        Gets the pose of the ball in the world frame.
        
        @return The pose of the ball.
        """
        
        #POSES = self.__get_pose_srv.call(model_name="cricket_ball", relative_entity_name="world")#ZZW 20221230.pose  self.__current_model_name
        #print("POSES ZZW 20221230::", POSES)
        #POSESS = self.__get_pose_srv.call(model_name="cricket_ball", relative_entity_name="world").pose
        #print("TYPE OF POSES ZZW 20221230:", type(POSESS))
        """
        Gets the pose of the ball in the world frame.

        @return The pose of the ball.
        """
        POSES = self.__get_pose_srv.call(model_name="cricket_ball", relative_entity_name="world").pose #ZZW 20221230 BALL POSE ORI:"world"

        print("POSES:", POSES)

        return POSES


SmartUR5e.get_object_pose