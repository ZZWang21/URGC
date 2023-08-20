#!/usr/bin/env python3
import rospy
from std_srvs.srv import Empty, EmptyRequest  #ZZW 20221229
from gazebo_msgs.srv import GetModelState, SetModelConfiguration, DeleteModel, \
    SpawnModel
from geometry_msgs.msg import Pose
from controller_manager_msgs.srv import SwitchController, SwitchControllerRequest
from moveit_msgs.msg import PlanningScene, PlanningSceneComponents
from moveit_msgs.srv import GetPlanningScene
import moveit_commander #import MoveGroupCommander
from moveit_commander import MoveGroupCommander
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
from moveit_msgs.msg import Constraints, OrientationConstraint
from tf_conversions import posemath
from PyKDL import *
import tf
import math
from geometry_msgs.msg import PoseStamped
from threading import Timer  ##!/usr/bin/python3

def __start_ctrl(self):
    rospy.loginfo("STARTING CONTROLLERS")
    self.__switch_ctrl.call(start_controllers=["arm_controller", "gripper_controller", "joint_state_controller"], 
                            stop_controllers=[], strictness=1)

rospy.wait_for_service("/controller_manager/switch_controller")#,10.0)
__switch_ctrl = rospy.ServiceProxy("/controller_manager/switch_controller", SwitchController)

rospy.wait_for_service("/gazebo/set_model_configuration")#,10.0)
__set_model = rospy.ServiceProxy("/gazebo/set_model_configuration", SetModelConfiguration)

rospy.wait_for_service("/gazebo/pause_physics",10.0)
__pause_physics = rospy.ServiceProxy("/gazebo/pause_physics", Empty)
#self.__pause_physics(EmptyRequest()) #ZZW 20221229 

#rospy.wait_for_service("/gazebo/unpause_physics")
__unpause_physics = rospy.ServiceProxy("/gazebo/unpause_physics", Empty)
#self.__unpause_physics(EmptyRequest()) #ZZW 20221229 


joint_names = ['elbow_joint', 'shoulder_lift_joint', 'shoulder_pan_joint', 'simple_gripper_gripper_finger1_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']#'simple_gripper_gripper_finger1_joint'
                        #'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 
                       #'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'
joint_positions = [-2.37605, -1.65243, -2.7991, 0.14, -0.70982, -4.708957, 5.0456189] #[1.88, -1.06, -0.3, 0.14, 0.76, -4.7, 1.24] #[-1.0, 0.3, 1.0, -0.5, 0.5, 0.0] #[-1.5, 0.3, 1.2, 0.0, -0.5, -1.5, 0.0]
        
        #3.71887, 4.7149, -1.9282
response = __set_model("ur5", "robot_description", joint_names, joint_positions)
if response.success:
    print("Model configuration set successfully. reset_only_pose")
    #print("CURRENT REAL POSITION:", self.get_current_joint_state2())
else:
    print("Failed to set model configuration. reset_only_pose")


__unpause_physics.call()

