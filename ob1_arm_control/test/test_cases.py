#! /usr/bin/env python

###################################################
#                    ABOUT      
###################################################
"""
Author: Yousif El-Wishahy (yel-wishahy, yel.wishahy@gmail.com)
Last updated: Feb 25th, 2023

Test cases for the robotic arm control software of UBC Open Robotic's ob1 (op bots one) arm
Uses pytest.
"""

###################################################
#                    IMPORTS        
###################################################
import time
from moveit_commander.conversions import pose_to_list
import numpy as np
from geometry_msgs.msg import PoseStamped, Quaternion
from moveit_commander.planning_scene_interface import CollisionObject
import rospkg
import pytest
from test_helpers import *
import random
from copy import deepcopy

from ob1_arm_control import ArmCommander
from ob1_arm_control.tf_helpers import broadcast_point, broadcast_pose

###################################################
#                    TEST PARAMETERS
###################################################
GOAL_TOLERANCE = 0.001 #elementwise distance and angles for orientation
JOINT_TOLERANCE = 0.001 #element wise angles
DIST_TOLERANCE = 0.01 #distance in m
SUCCESS_RATE_TOLERANCE = 0.90 #percentage of tests that pass

###################################################
#                    TEST CASES               
###################################################

################### UNIT TESTS ######################
class UnitTests:
    def test_allclose_array(self):
        arr1 = np.random.rand(6)[0]
        arr2 = deepcopy(arr1)
        assert all_close(arr1,arr2,0)

    def test_allclose_pose(self, arm_commander):
        p1 = arm_commander.arm_mvgroup.get_random_pose()
        p2 = deepcopy(p1)
        assert all_close(p1,p2,0)

    def test_allclose_point(self, arm_commander):
        p1 = arm_commander.arm_mvgroup.get_random_pose().pose.position
        p2 = deepcopy(p1)
        assert all_close(p1,p2,0)

    def test_allclose_quaternion(self, arm_commander):
        q1 = arm_commander.arm_mvgroup.get_random_pose().pose.orientation
        q2 = deepcopy(q1)
        assert all_close(q1,q2,0)

################### GO TO TARGET TESTS  ######################
# with default moveit ik planner
class TestGoTarget:
    def test_go_joints_allclose(self, arm_commander, log, setup_teardown):
        target = arm_commander.arm_mvgroup.get_random_joint_values()
        log.info(target)
        res = arm_commander.go_joint(target)
        assert res , "motion planning failed"
        assert all_close(target, arm_commander.arm_mvgroup.get_current_joint_values(), JOINT_TOLERANCE)

    def test_go_position_dist(self, arm_commander, log, setup_teardown):
        target = arm_commander.arm_mvgroup.get_random_pose().pose.position
        broadcast_point(arm_commander.tf_broadcaster,target,'target','world')
        res = arm_commander.go_position(target)
        assert res , "motion planning failed"
        assert_dist(arm_commander.get_end_effector_pose().pose.position, target, DIST_TOLERANCE)

    def test_go_position_allclose(self, arm_commander, log, setup_teardown):
        target = arm_commander.arm_mvgroup.get_random_pose().pose.position
        broadcast_point(arm_commander.tf_broadcaster,target,'target','world')
        res = arm_commander.go_position(target)
        assert res , "motion planning failed"
        assert all_close(target, arm_commander.get_end_effector_pose().pose.position, GOAL_TOLERANCE)
    
    def test_go_pose_dist(self, arm_commander, log, setup_teardown):
        target = arm_commander.arm_mvgroup.get_random_pose()
        broadcast_pose(arm_commander.tf_broadcaster,target.pose,'target','world')
        res = arm_commander.go_pose(target)
        assert res , "motion planning failed"
        assert_dist(arm_commander.get_end_effector_pose(), target, DIST_TOLERANCE)

    def test_go_pose_allclose(self, arm_commander, log, setup_teardown):
        target = arm_commander.arm_mvgroup.get_random_pose()
        broadcast_pose(arm_commander.tf_broadcaster,target.pose,'target','world')
        res = arm_commander.go_pose(target)
        assert res , "motion planning failed"
        assert all_close(target, arm_commander.get_end_effector_pose(), GOAL_TOLERANCE)

################### OBJECT MANIPULATION TESTS ######################
class TestPickSphere:
    @pytest.mark.parametrize("sphere_radius", [0.03])
    def test_go_sphere(self, arm_commander:ArmCommander, log, setup_teardown, sphere_radius):
        spawn_random_sphere(arm_commander, sphere_radius)
        objs = arm_commander.scene.get_objects()
        assert len(objs) > 0, "Could not find any scene objects"
        obj:CollisionObject = list(objs.items())[0][1]
        broadcast_pose(arm_commander.tf_broadcaster,obj.pose,'target','world')
        res = arm_commander.go_position(obj.pose.position)
        assert res , "motion planning failed"
        assert_dist(arm_commander.get_end_effector_pose().pose.position, obj.pose.position, DIST_TOLERANCE)

    @pytest.mark.parametrize("sphere_radius", [0.03])
    def test_pick_sphere(self, arm_commander, log, setup_teardown, sphere_radius):
        spawn_random_sphere(arm_commander, sphere_radius)
        objs = arm_commander.scene.get_objects()
        assert len(objs) > 0, "Could not find any scene objects"
        obj:CollisionObject = list(objs.items())[0][1]
        broadcast_pose(arm_commander.tf_broadcaster,obj.pose,'target','world')
        assert arm_commander.pick_object(obj)

    @pytest.mark.parametrize("sphere_radius", [0.03])
    def test_pick_and_move_sphere(self, arm_commander:ArmCommander, log, setup_teardown, sphere_radius):
        spawn_random_sphere(arm_commander, sphere_radius)
        objs = arm_commander.scene.get_objects()
        assert len(objs) >= 0, 'could not find any scene objects'
        obj:CollisionObject = list(objs.items())[0][1]
        broadcast_pose(arm_commander.tf_broadcaster,obj.pose,'target','world')

        assert arm_commander.pick_object(obj), 'failed to pick object'
        
        assert arm_commander.go_position(arm_commander.arm_mvgroup.get_random_pose().pose.position), 'failed to move arm after attaching object'

        assert arm_commander.detach_object(obj), 'failed to detach object'

        assert arm_commander.open_gripper(), 'failed to open gripper after detaching object'

    def test_pick_cylinder_and_move(self, arm_commander:ArmCommander, log, setup_teardown):
        p = PoseStamped()
        p.header.frame_id = arm_commander.robot.get_planning_frame()
        p.pose.position = arm_commander.arm_mvgroup.get_random_pose().pose.position
        p.pose.orientation.w = 1
        arm_commander.scene.add_cylinder('cylinder', p, height=0.15, radius=0.03)

        time.sleep(2)

        obj = arm_commander.get_object('cylinder')
        assert obj!=None, 'could not find any scene objects'
        broadcast_pose(arm_commander.tf_broadcaster,obj.pose,'target','world')

        assert arm_commander.pick_object(obj), 'failed to pick object'
        
        assert arm_commander.go_position(arm_commander.arm_mvgroup.get_random_pose().pose.position), 'failed to move arm after attaching object'

        assert arm_commander.detach_object(obj), 'failed to detach object'

        assert arm_commander.open_gripper(), 'failed to open gripper after detaching object'
