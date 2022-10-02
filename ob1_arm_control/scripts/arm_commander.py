#!/usr/bin/env python3.8
from __future__ import print_function
import pickle
import sys
from turtle import pos
import rospy
import moveit_commander
from moveit_commander.conversions import pose_to_list
import numpy as np
from math import pi, tau, dist, fabs, cos
import geometry_msgs
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped, Pose, Point
from moveit_commander import MoveGroupCommander,RobotCommander, PlanningSceneInterface
from shape_msgs.msg import SolidPrimitive
from moveit_commander.planning_scene_interface import CollisionObject
from moveit_msgs.msg import Grasp, GripperTranslation, MoveItErrorCodes,DisplayTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from tf.transformations import quaternion_from_euler
from copy import deepcopy
import rospkg
from ikpoints import IKPoints

# docs: https://github.com/ros-planning/moveit/tree/ee48dc5cedc981d0869352aa3db0b41469c2735c
# Author: Yousif El-Wishahy

##############################
# arm command class to be called by robot loop
##############################
class ArmCommander(object):
    _current_plan = None # current joint trajectory plan
    _current_pose_goal : PoseStamped = None #current pose goal 
    _sample_time_out   = 5             # time out in seconds for Inverse Kinematic search
    _sample_attempts   = 5             # num of planning attempts 
    _goal_tolerance    = 0.001
    _joint_tolerances = []
    _num_joints = 0

    #group and link anmes
    ARM_GROUP_NAME = "arm"
    GRIPPER_GROUP_NAME = "gripper"
    GRIPPER_LINK_NAME = "main_assembly"

    #reference frames
    ARM_REF_FRAME = "base_link"
    GRIPPER_REF_FRAME = "main_assembly"
    WORLD_REF_FRAME = "world"

    PACKAGE_PATH = rospkg.RosPack().get_path('ob1_arm_control')
    IKPOINTS_DATA_FILE_PATH = PACKAGE_PATH + "/data/5k_ikpoints_data.pickle"
    ikpoints:IKPoints = None
 
    def __init__(self, sample_time_out=5, sample_attempts=5, goal_tolerance=0.05):
        '''
        @brief init arm command object, moveit commander, scene and movegroups for arm and arm gripper
        '''
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node("arm_commander", anonymous=True)

        self.robot:RobotCommander = moveit_commander.RobotCommander()
        self.scene:PlanningSceneInterface = moveit_commander.PlanningSceneInterface()

        #init arm move group 
        self.arm_mvgroup:MoveGroupCommander = moveit_commander.MoveGroupCommander(self.ARM_GROUP_NAME)
        self.arm_mvgroup.set_planning_time(sample_time_out)
        self.arm_mvgroup.set_num_planning_attempts(sample_attempts)
        self.arm_mvgroup.set_goal_tolerance(goal_tolerance)
        self.arm_mvgroup.set_max_velocity_scaling_factor(1)
        self.arm_mvgroup.set_max_acceleration_scaling_factor(0.3)

        self._num_joints = len(self.robot.get_active_joint_names())

        self._goal_tolerance = goal_tolerance
        self._sample_attempts = sample_attempts
        self._sample_time_out = sample_time_out

        # self.arm_mvgroup.set_workspace([-50,-50,-50,50,50,50])
        
        # self.arm_mvgroup.set_end_effector_link(self.GRIPPER_REF_FRAME)
        # self.arm_mvgroup.set_pose_reference_frame(self.GRIPPER_REF_FRAME)

        # self.gripper_mvgroup:MoveGroupCommander = moveit_commander.MoveGroupCommander(self.GRIPPER_GROUP_NAME)
        
        #debug print
        print("============ Reference frame: %s" % self.arm_mvgroup.get_planning_frame())

        print("============ Pose Reference frame: %s" % self.arm_mvgroup.get_pose_reference_frame())

        eef_link = self.arm_mvgroup.get_end_effector_link()
        print("============ End effector: %s" % eef_link)

        print("============ Robot Groups:", self.robot.get_group_names())

        print("============ Robot Links:", self.robot.get_link_names())

        print("============ Robot Joints:", self.robot.get_active_joint_names())

        print("============ Printing robot state")
        print(self.robot.get_current_state())

        print("============ Printing MoveIt interface description")
        print(self.arm_mvgroup.get_interface_description())

        for joint_name in self.arm_mvgroup.get_active_joints():
            joint = self.robot.get_joint(joint_name)
            self._joint_tolerances.append((joint.bounds()[0],joint.bounds()[1]))

        self.ikpoints = IKPoints(self.IKPOINTS_DATA_FILE_PATH)

    def register_gripper_edge_states(self):
        joint_vals_min = [0]
        joint_vals_max = [0]
        for joint_name in self.gripper_mvgroup.get_joints():
            joint = self.robot.get_joint(joint_name)
            joint_vals_min.append(joint.bounds()[0])
            joint_vals_max.append(joint.bounds()[1])

        #todo: test these remembered joint states
        self.gripper_mvgroup.remember_joint_values("open", values=joint_vals_max)
        self.gripper_mvgroup.remember_joint_values("close", values=joint_vals_min)

    def go_position(self, position_goal=None):
        """
        @brief makes the end effector of the arm go to a cartesian position (x,y,z)
        The position should be in the reference frame of the base link

        @param position_goal: can be list of [x,y,z] a Point, Pose, or PoseStamped objects
        The position will be extracted from any of these data types.

        @returns (execution result, position target [x,y,z] )
        """

        #handle different kind of inputs
        if position_goal is None:
            position_goal = [0,0,0]
            rand_pos = self.arm_mvgroup.get_random_pose().pose.position
            position_goal[0] = rand_pos.x
            position_goal[1] = rand_pos.y
            position_goal[2] = rand_pos.z
        elif type(position_goal) is not list:
            position_goal = convert_to_list(position_goal)
        
        self.arm_mvgroup.clear_pose_targets()
        self.arm_mvgroup.set_position_target(position_goal, self.GRIPPER_LINK_NAME)
        plan = self.arm_mvgroup.plan()
        res = plan[0]

        if plan[0]:
            res = self.arm_mvgroup.go()
            self.arm_mvgroup.stop()
            self.arm_mvgroup.clear_pose_targets()

        self._current_plan = plan
        return res, position_goal

    def go_scene_object(self, object:CollisionObject, attempts=10, d=0.1):
        """
        @brief makes the end effector of the arm go to the proximity of a scene object
        Finds optimal joint target based on ik points data base
        The position of the object should be in the reference frame of the base link
        Proximity is based on the closest reachable ikpoint to the scene object position

        @param object: scene object with type moveit_commander.planning_scene_interface.CollisionObject
        
        @param(optional,defuault=10) attempts: number of different points to attempt near scene object

        @param(optiona,default=0.1) d: distance resolution to scale vectors by when attempting to find nearby

        @returns (execution result, object pose, successful_joint_target [j0,j1,j2,...] | None)
        """
    
        pt_obj = np.array(convert_to_list(object.pose.position))
        shape:SolidPrimitive = object.primitives[0]

        if self.ikpoints.in_range(pt_obj,self._goal_tolerance):

            joint_targets = self.ikpoints.get_nearest_joint_targets(pt_obj,num_pts=attempts)
            successful_joint_target = None

            for joints in joint_targets:
                self.arm_mvgroup.set_joint_value_target(joints)
                plan = self.arm_mvgroup.plan()
                if plan[0]:
                    successful_joint_target = joints
                    break
            
            if not plan[0]:
                for i in range(attempts):
                    p2 = pt_obj * (1 - d*i)
                    joints = self.ikpoints.get_nearest_joint_targets(p2)
                    self.arm_mvgroup.set_joint_value_target(joints)
                    plan = self.arm_mvgroup.plan()
                    if plan[0]:
                        successful_joint_target = joints
                        break
            
            if plan[0]:
                res = self.arm_mvgroup.go()
                self.arm_mvgroup.stop()
                self._current_plan = plan
                return res, object.pose, successful_joint_target 

        print("Could not complete motion planning for scene object")
        return False, object.pose, None
        
    def go_position_ikpoints(self, position_goal=None):
        """
        @brief makes the end effector of the arm go to a cartesian position (x,y,z)
        Finds optimal joint target based on ik points data base
        The position should be in the reference frame of the base link

        @param position_goal: can be list of [x,y,z] a Point, Pose, or PoseStamped objects
        The position will be extracted from any of these data types.

        **IF position_goal is None, a random position_goal will be selected and the nearest ikpoint will be matched to it

        @returns (execution result, position target [x,y,z], joint_target [j0,j1,j2,...])
        """

        #handle different kind of inputs
        if position_goal is None:
            position_goal = [0,0,0]
            rand_pos = self.arm_mvgroup.get_random_pose().pose.position
            position_goal[0] = rand_pos.x
            position_goal[1] = rand_pos.y
            position_goal[2] = rand_pos.z
        elif type(position_goal) is not list:
            position_goal = convert_to_list(position_goal)

        joint_target = self.ikpoints.get_nearest_joint_targets(np.array(position_goal))
        #res = (execution result, joint_target)
        res = self.go_joint(joint_target)
        return res[0], position_goal, res[1]
        
    def go_pose(self, pose_goal:PoseStamped=None):
        '''
        @brief Makes the end effector of the arm go to a pose, 
        blocks until either inverse kinematics fails 
        to find path or it has reached the goal within tolerace.

        @param pose_goal : geometry_msgs\Pose contains desired 
        position and orientation for gripper

        if pose_goal param is not passed, random pose goal will be generated

        @returns (execution result, PoseStamped target)
        '''

        if pose_goal == None:
            self._current_pose_goal = self.arm_mvgroup.get_random_pose()
        else:
            self._current_pose_goal = pose_goal

        if self.robot.get_planning_frame() != self._current_pose_goal.header.frame_id:
            print("Incorrect planning frame for pose goal, got %s instead of %s \n" \
                    %(self._current_pose_goal.header.frame_id,self.robot.get_planning_frame()))
            self._current_pose_goal = None
            return False, None

        self._current_plan = self.arm_mvgroup.plan(self._current_pose_goal.pose)
        res = self._current_plan[0]
        if (self._current_plan[0]):
            res = self.arm_mvgroup.go(joints=None, wait=True)
            self.arm_mvgroup.stop()
            self.arm_mvgroup.clear_pose_targets()
        return res, self._current_pose_goal

    def go_joint(self, joints=None):
        """
        @brief Makes the group's joints go to joint angles specified by goal,
        blocks until goal is reached within tolerance or exit if goal joint
        angles is impossible.

        @param group (MoveGroupCommander) : Move group object , either arm or gripper

        @param goal (JointState): JointState object that specifies joint angle goals

        @returns (execution result, joint_target [j0,j1,j2,...])
        """
        if type(joints) is not list or len(joints) != self._num_joints:
            if type(joints) is np.ndarray:
                joints = joints.tolist()
            else:
                print('getting random joint values')
                joints = self.arm_mvgroup.get_random_joint_values()

        self.arm_mvgroup.set_joint_value_target(joints)
        plan = self.arm_mvgroup.plan()
        res = plan[0]

        if plan[0]:
            res = self.arm_mvgroup.go()
            self.arm_mvgroup.stop()
            self.arm_mvgroup.clear_pose_targets()
        self._current_plan = plan
        return res, joints
    
    def get_end_effector_pose(self):
        """
        Returns PoseStamped object of end effector of arm move group
        """
        return self.robot.get_link(self.arm_mvgroup.get_end_effector_link()).pose()

    def print_robot_cartesian_state(self):
        """
        Prints cartesian pose state of all robot links
        """
        print('ROBOT CARTESIAN STATE\n')
        for link_name in self.robot.get_link_names():
            link = self.robot.get_link(link_name)
            print(link.name(),'\n',link.pose().pose,'\n',link.pose().header.frame_id)

    # def compare_eef_pose_states(self):
    #     """
    #     @brief print goal pose and actual pose and if it meets self.goal_tolerance
    #     """
    #     if self._current_plan[0]:
    #         print('PLANNING FAILED')
    #     goal = self._current_pose_goal
    #     actual = self.get_end_effector_pose()
    #     result = all_close(goal,actual,self._goal_tolerance)

    #     if goal.header.frame_id == actual.header.frame_id:
    #         print('\n\n')
    #         print("End effector goal:\n",goal)
    #         print('\n')
    #         print("End effector result:\n",actual)
    #         print('\n\n')
    #         print('Goal within tolerance: %s : %s' % (self._goal_tolerance,result))
    #     else:
    #         print("pose goal not in correct reference frame, %s instead of %s" %(goal.header.frame_id,actual.header.frame_id))

    #     return result

def all_close(goal, actual, tolerance):
    """
    From: https://github.com/ros-planning/moveit_tutorials/blob/master/doc/move_group_python_interface/scripts/move_group_python_interface_tutorial.py
    Convenience method for testing if the values in two lists are within a tolerance of each other.
    For Pose and PoseStamped inputs, the angle between the two quaternions is compared (the angle
    between the identical orientations q and -q is calculated correctly).
    @param: goal       A list of floats, a Pose or a PoseStamped
    @param: actual     A list of floats, a Pose or a PoseStamped
    @param: tolerance  A float
    @returns: bool
    """
    if type(goal) is list:
        for index in range(len(goal)):
            if abs(actual[index] - goal[index]) > tolerance:
                return False

    elif type(goal) is geometry_msgs.msg.PoseStamped:
        return all_close(goal.pose, actual.pose, tolerance)

    elif type(goal) is geometry_msgs.msg.Pose:
        x0, y0, z0, qx0, qy0, qz0, qw0 = pose_to_list(actual)
        x1, y1, z1, qx1, qy1, qz1, qw1 = pose_to_list(goal)
        # Euclidean distance
        d = dist((x1, y1, z1), (x0, y0, z0))
        # phi = angle between orientations
        cos_phi_half = fabs(qx0 * qx1 + qy0 * qy1 + qz0 * qz1 + qw0 * qw1)
        return d <= tolerance and cos_phi_half >= cos(tolerance / 2.0)

    return True

def convert_to_list(obj):
    """
    @brief helper function to convert geometry messages to list [x,y,z]

    returns none if invalid input object
    """
    if type(obj) is Point:
        return [obj.x,obj.y,obj.z]
    elif type(obj) is Pose:
        return [ obj.position.x,
                 obj.position.y,
                 obj.position.z ]
    elif type(obj) is PoseStamped:
        return [ obj.pose.position.x,
                 obj.pose.position.y,
                 obj.pose.position.z ]
    elif type(obj) is np.ndarray:
        return obj.tolist()
    elif type(obj) is list:
        return obj
    else:
        print(obj)
        raise ValueError("Invalid input to convert_to_list funtion")
