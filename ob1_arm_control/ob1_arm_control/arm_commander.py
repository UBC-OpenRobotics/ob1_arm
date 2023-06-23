#! /usr/bin/env python
"""
Author: Yousif El-Wishahy
Email: yel.wishahy@gmail.com 

Module containing arm commander class for controlling UBC Open Robotics' Ob1 Arm
"""
###########################################################33
import sys
import rospy
import moveit_commander
import numpy as np
from geometry_msgs.msg import PoseStamped, Pose, Point, PointStamped, Quaternion
from moveit_commander import MoveGroupCommander,RobotCommander, PlanningSceneInterface
from moveit_msgs.msg import CollisionObject
import time
import tf

from .tf_helpers import *


class ArmCommander:
    _current_plan = None # current joint trajectory plan
    _current_pose_goal : PoseStamped = None #current pose goal 
    _arm_joint_limits = []
    _gripper_joint_limits = []
    _num_joints = 0

    def __init__(self, sample_timeout=0.1, sample_attempts=5, goal_tolerance=0.001, joint_tolerance=0.001, is_interbotix=False):
        '''
        @brief init arm command object, moveit commander, scene and movegroups for arm and arm gripper
        '''
        if is_interbotix:
            # group and link names for interbotix
            self.ROBOT_DESCRIPTION = 'rx150/robot_description'
            self.ARM_GROUP_NAME = "interbotix_arm"
            self.GRIPPER_GROUP_NAME = "interbotix_gripper"
            self.EEF_LINK_NAME = ""
            self.GRIPPER_BASE_LINK_NAME = ""
            self.GRIPPER_LCLAW_LINK_NAME = ""
            self.GRIPPER_RCLAW_LINK_NAME = ""
            self.WORLD_REF_FRAME = "world"
            self.NS = ''
        else:    
            #group and link names for ob1arm
            self.ROBOT_DESCRIPTION = 'robot_description'
            self.ARM_GROUP_NAME = "arm"
            self.GRIPPER_GROUP_NAME = "gripper"
            self.EEF_LINK_NAME = "ob1_arm_eef_link"
            self.GRIPPER_BASE_LINK_NAME = "ob1_arm_gripper_base_link"
            self.GRIPPER_LCLAW_LINK_NAME = "ob1_arm_lclaw_link"
            self.GRIPPER_RCLAW_LINK_NAME = "ob1_arm_rclaw_link"
            self.WORLD_REF_FRAME = "world"
            self.NS = ''

        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node("arm_commander")
        rospy.loginfo("Initialized arm_commander node")

        self._sample_timeout = sample_timeout
        self._sample_attempts = sample_attempts
        self._goal_tolerance = goal_tolerance
        self._joint_tolerance = joint_tolerance

        self.robot:RobotCommander = moveit_commander.RobotCommander(robot_description = self.ROBOT_DESCRIPTION, ns = self.NS)
        self.scene:PlanningSceneInterface = moveit_commander.PlanningSceneInterface(ns=self.NS)
        self.tf_listener = tf.TransformListener()
        self.tf_broadcaster = tf.TransformBroadcaster()

        #init arm move group 
        self.arm_mvgroup:MoveGroupCommander = moveit_commander.MoveGroupCommander(self.ARM_GROUP_NAME, ns=self.NS)
        
        #init gripper move group
        self.gripper_mvgroup:MoveGroupCommander = moveit_commander.MoveGroupCommander(self.GRIPPER_GROUP_NAME, ns=self.NS)

        self._num_joints = len(self.arm_mvgroup.get_random_joint_values())

        def set_mvgrp_params(mv_grp:MoveGroupCommander):
            mv_grp.set_planning_time(self._sample_timeout)
            mv_grp.set_num_planning_attempts(self._sample_attempts)
            mv_grp.set_goal_tolerance(self._goal_tolerance)
            mv_grp.set_goal_joint_tolerance(self._joint_tolerance)
            rospy.loginfo("==== Initialized move group instance: %s" % mv_grp.get_name())
            rospy.loginfo("==== Sample time out: %s s" % self._sample_timeout)
            rospy.loginfo("==== Sample attempts: %d " % self._sample_attempts)
            rospy.loginfo("==== Goal Tolerance: %f m" % self._goal_tolerance)
            rospy.loginfo("==== Joint Tolerance: %f m" % self._joint_tolerance)
        set_mvgrp_params(self.arm_mvgroup)
        set_mvgrp_params(self.gripper_mvgroup)
        
        rospy.loginfo("==== Planning Reference Frame: %s" % self.arm_mvgroup.get_planning_frame())

        rospy.loginfo("==== Pose Reference Frame: %s" % self.arm_mvgroup.get_pose_reference_frame())

        rospy.loginfo("==== End Effector: %s" % self.arm_mvgroup.get_end_effector_link())

        rospy.loginfo("==== Robot Groups: %s" % self.robot.get_group_names() )

        rospy.loginfo("==== Robot Links ====\n\n%s\n\n" % self.robot.get_link_names())

        rospy.loginfo("==== Robot Joints ====\n\n%s\n\n" % self.robot.get_active_joint_names())

        rospy.loginfo("==== MoveIt Interface Description ====\n\n%s\n" % self.arm_mvgroup.get_interface_description())

        rospy.loginfo("==== Storing Joint Limits")
        for joint_name in self.arm_mvgroup.get_active_joints():
            joint = self.robot.get_joint(joint_name)
            self._arm_joint_limits.append((joint.bounds()[0],joint.bounds()[1]))
        for joint_name in self.gripper_mvgroup.get_active_joints():
            joint = self.robot.get_joint(joint_name)
            self._gripper_joint_limits.append((joint.bounds()[0],joint.bounds()[1]))

    def _check_pose_goal(self,pose):
        if type(pose) is PoseStamped:
            if pose.header.frame_id == self.arm_mvgroup.get_planning_frame():
                return True
        return False

    def _enforce_joint_limits(self,joints):
        """
        @brief helper function to ensure joint limits

        @param joints, list[floats] of joint values

        @return list of joint values clamped to their limits (as defined by the urdf)
        """
        if len(self._arm_joint_limits) < 0 :
            return joints
        for i in range(self._num_joints):
            joints[i] = np.clip(joints[i], self._arm_joint_limits[i][0], self._arm_joint_limits[i][1])
            print(joints[i])
        return joints

    def _check_joint_limits(self,joints):
        """
        @brief check that joint values do not exceed limits

        @param joints, list[floats] of joint values

        @return bool
        """
        if len(joints) != self._num_joints:
            return False
        for i in range(self._num_joints):
            if joints[i] < self._arm_joint_limits[i][0] or joints[i] > self._arm_joint_limits[i][1]:
                return False
        return True

    def get_end_effector_pose(self):
        """
        Returns PoseStamped object of end effector of arm move group
        """
        return self.robot.get_link(self.arm_mvgroup.get_end_effector_link()).pose()

    def go_joint(self, joints:list) -> bool:
        """
        @brief Makes the group's joints go to joint angles specified by goal,
        blocks until goal is reached within tolerance or exit if goal joint
        angles is impossible.

        @param joints: list[float] of joint target angles

        @returns (execution result, joint_target [j0,j1,j2,...] | None)
        """
        if type(joints) is not list:
            joints = convert_to_list(joints)

        if len(joints) != self._num_joints:
            raise ValueError("joints list has incorrect length (!=%d)"%self._num_joints)

        self.arm_mvgroup.set_joint_value_target(joints)
        result = self.arm_mvgroup.go()
        self.arm_mvgroup.stop()
        return result
    
    def go_position(self, position_goal) -> bool:
        """
        @brief makes the end effector of the arm go to a cartesian position (x,y,z)
        The position should be in the reference frame of the base link

        @param position_goal: can be list of [x,y,z] a Point, Pose, or PoseStamped objects
        The position will be extracted from any of these data types.

        @returns (execution result, position target [x,y,z] )
        """

        if type(position_goal) is not list:
            position_goal = convert_to_list(position_goal)
            if len(position_goal) != 3:
                raise ValueError("Position goal list has invalid length (!=3)")

        self.arm_mvgroup.set_position_target(position_goal)
        res = self.arm_mvgroup.go()
        self.arm_mvgroup.stop()
    
        return res
        
    def go_pose(self, pose_goal:PoseStamped) -> bool:
        '''
        @brief Makes the end effector of the arm go to a pose, 
        blocks until either inverse kinematics fails 
        to find path or it has reached the goal within tolerace.

        @param pose_goal : geometry_msgs\Pose contains desired 
        position and orientation for gripper

        if pose_goal param is not passed, random pose goal will be generated

        @returns execution result
        '''

        if type(pose_goal) is not PoseStamped:
            raise ValueError("Input goal is not a PoseStamped")
        if self.robot.get_planning_frame() != pose_goal.header.frame_id:
            raise ValueError("Incorrect planning frame for pose goal, got %s instead of %s \n" \
                    %(self._current_pose_goal.header.frame_id,self.robot.get_planning_frame()))

        self._current_pose_goal = pose_goal

        self.arm_mvgroup.set_pose_target(pose_goal)
        # self.arm_mvgroup.set_joint_value_target(pose_goal) #read the function docs, this should work
        res = self.arm_mvgroup.go()
        self.arm_mvgroup.stop()
        self.arm_mvgroup.clear_pose_targets()

        return res

    def get_object(self, object_id:str):
        """
        @brief Get collision object from the planning scene based on id string

        @return Moveit_msgs/CollisionObject if found | None if not found
        """
        objs = self.scene.get_objects()
        for o in list(objs.items()):
            obj:CollisionObject = o[1]
            # shape:SolidPrimitive = obj.primitives[0]
            if obj.id == object_id:
                return obj
        print('could not find obj in %s' %objs)
        return None

    def close_gripper(self):
        """
        @brief close gripper to previously defined 'close' joint values
        """
        joints_close = list(self.gripper_mvgroup.get_named_target_values("close").values())
        return self.gripper_mvgroup.go(joints_close)

    def open_gripper(self):
        """
        @brief open gripper to previously defined 'open' joint values
        """
        joints_close = list(self.gripper_mvgroup.get_named_target_values("open").values())
        return self.gripper_mvgroup.go(joints_close)

    def is_object_attached(self, object_id:str):
        """
        @brief check if object with id object_id is attached to the arm/gripper in the planning scene
        """
        res:bool = False 
        if self.scene.get_attached_objects([object_id]):
            #empty dictionaries eval to false in python
            res = True
        return res

    def attach_object(self, object:CollisionObject):
        """
        @brief attach CollisionObject object to the gripper in the planning scene
        """
        touch_links = self.robot.get_link_names(group=self.GRIPPER_GROUP_NAME)
        self.scene.attach_box(self.EEF_LINK_NAME, object.id, touch_links=touch_links)
        time.sleep(2) #delay for scene update
        return self.is_object_attached(object.id)
    
    def detach_object(self, object:CollisionObject):
        """
        @brief detach CollisionObject object from the gripper in the planning scene
        """
        self.scene.remove_attached_object(self.EEF_LINK_NAME, name=object.id)
        time.sleep(2) #delay for scene update
        return not self.is_object_attached(object.id)

    def pick_object(self, object:CollisionObject):
        def close_gripper_around_object():
            self.gripper_mvgroup.set_planning_time(0.1)
            self.gripper_mvgroup.set_num_planning_attempts(1)
            joints_open = list(self.gripper_mvgroup.get_named_target_values("open").values())
            joints_close = list(self.gripper_mvgroup.get_named_target_values("close").values())
            js_0 = np.linspace(joints_close[0], joints_open[0], 10)
            js_1 = np.linspace(joints_close[1], joints_open[1], 10)
            for a0 in js_0:
                for a1 in js_1:
                    self.gripper_mvgroup.set_joint_value_target([a0,a1])
                    plan = self.gripper_mvgroup.plan()
                    if plan[0]:
                        if self.gripper_mvgroup.go():
                            return True
            return False
        
        print("Picking object: %s" % object.id)
        self.open_gripper()

        res = self.go_position(object.pose.position)
        if res:
            res = close_gripper_around_object()
            if res:
                res = self.attach_object(object)
        return res
