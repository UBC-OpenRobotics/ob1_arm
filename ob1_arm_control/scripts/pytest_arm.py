#!/usr/bin/env python3.8
from __future__ import print_function
from operator import itemgetter
import pickle
import time
from arm_commander import ArmCommander, convert_to_list
import sys
import rospy
import moveit_commander
from moveit_commander.conversions import pose_to_list
import numpy as np
from math import pi, tau, dist, fabs, cos
import geometry_msgs
from geometry_msgs.msg import PoseStamped, Pose, Vector3Stamped 
from moveit_commander import MoveGroupCommander,RobotCommander, PlanningSceneInterface
from moveit_commander.planning_scene_interface import CollisionObject
from moveit_msgs.msg import Grasp, GripperTranslation, MoveItErrorCodes,DisplayTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from tf.transformations import quaternion_from_euler
from copy import deepcopy
import rospkg
import pytest
import logging

##TEST PARAMS
# rp = rospkg.RosPack()
# PACKAGE_PATH = rp.get_path('ob1_arm_control')
# DATA_FILE_PATH = PACKAGE_PATH+"/data/5k_pose_data.pickle"

PLANNING_TOLERANCE = 0.05 #distance in metres
SUCCESS_RATE_TOLERANCE = 0.80 #percentage of tests that pass

iterations = 10

log = logging.getLogger(__name__)

arm_commander = ArmCommander(sample_time_out=5,goal_tolerance=0.01)
log.info("Clearing scene..")
arm_commander.scene.clear()

def dist(p1,p2):
    p1 = np.array(convert_to_list(p1))
    p2 = np.array(convert_to_list(p2))
    return np.linalg.norm(p1-p2)

def assert_ik_result(current_state,target,tolerance:float):
    if type(target) is PoseStamped and type(current_state) is PoseStamped:
        assert target.header.frame_id == current_state.header.frame_id , "transform frame mismatch"
    d = dist(current_state,target)
    assert d <= tolerance, "Distance from target is too far, %f cm" % (d*100)

def compare_ik_result(current_state,target,tolerance:float):
    if type(target) is PoseStamped and type(current_state) is PoseStamped:
        if target.header.frame_id != current_state.header.frame_id:
            return False, "transform frame mismatch"
    d = dist(current_state,target)
    if d <= tolerance:
        return True, "Motion planning tolerance success"
    else:
        return False, "Distance from target is too far, %f cm" % (d*100)

def test_go_rand_pose():
    log.info("Starting go_rand_pose test")
    log.info("Going to random pose...")
    res, target = arm_commander.go_pose()
    assert res , "motion planning failed"
    log.info("Boolean result: %s" % res)
    log.info("Target type: %s" % type(target))
    assert_ik_result(arm_commander.get_end_effector_pose(),target,PLANNING_TOLERANCE)

def test_go_rand_position():
    log.info("Starting go_rand_position test")
    log.info("Going to random position...")
    res, target = arm_commander.go_position()
    assert res , "motion planning failed"
    log.info("Boolean result: %s" % res)
    log.info("Target type: %s" % type(target))
    assert_ik_result(arm_commander.get_end_effector_pose(),target,PLANNING_TOLERANCE)

def test_go_rand_joint_target():
    log.info("Starting go_rand_joint_target test")
    log.info("Going to random joint target...")
    res, target = arm_commander.go_joint()
    assert res , "motion planning failed"
    log.info("Boolean result: %s" % res)
    log.info("Target type: %s" % type(target))
    assert_ik_result(arm_commander.arm_mvgroup.get_current_joint_values(),target,PLANNING_TOLERANCE)

def test_go_rand_ikpoint():
    log.info("Starting go_rand_ikpoint test")
    log.info("Generating random point and matching nearest ikpoint...")
    res, target, joint_target = arm_commander.go_position_ikpoints()
    assert res , "motion planning failed"
    log.info("Boolean result: %s" % res)
    log.info("Target type: %s" % type(target))
    assert_ik_result(arm_commander.get_end_effector_pose(),target,PLANNING_TOLERANCE)

def test_go_rand_reachable_scene_object():
    log.info("Starting go_rand_reachable_scene_object test")

    log.info("Clearing scene..")
    arm_commander.scene.clear()

    log.info("Generating and placing scene object..")
    p = PoseStamped()
    p.header.frame_id = arm_commander.robot.get_planning_frame()
    rand_pos = arm_commander.ikpoints.get_rand_point()
    p.pose.position.x = rand_pos[0]
    p.pose.position.y = rand_pos[1]
    p.pose.position.z = rand_pos[2]
    p.pose.orientation.w = 1
    arm_commander.scene.add_sphere("sphere",p,0.1)

    # log.info("Detecting scene objects..")
    # objs = arm_commander.scene.get_objects()
    # assert len(objs) > 0, "Could not find any scene objects"
    # obj:CollisionObject = list(objs.items())[0][1]
    # pos = [obj.pose.position.x,obj.pose.position.y,obj.pose.position.z]
    pos = rand_pos

    log.info("Motion planning to scene object")
    res, target, joint_target = arm_commander.go_position_ikpoints(pos)

    assert res , "motion planning failed"

    log.info("Boolean result: %s" % res)
    log.info("Target type: %s" % type(target))
    assert_ik_result(arm_commander.get_end_effector_pose(),target,PLANNING_TOLERANCE)

#TODO IMPROVE LOGIC FOR SMART FINDING SCENE OBJECTS
def test_go_rand_scene_object_smart():
    log.info("Starting go_rand_scene_object test")

    log.info("Clearing scene..")
    arm_commander.scene.clear()

    log.info("Generating and placing scene object..")
    p = PoseStamped()
    p.header.frame_id = arm_commander.robot.get_planning_frame()
    rand_pos = arm_commander.ikpoints.get_rand_point()
    p.pose.position.x = rand_pos[0]
    p.pose.position.y = rand_pos[1]
    p.pose.position.z = rand_pos[2]
    p.pose.orientation.w = 1
    arm_commander.scene.add_sphere("sphere",p,0.1)

    # log.info("Detecting scene objects..")
    # objs = arm_commander.scene.get_objects()

    # assert len(objs) > 0, "Could not find any scene objects"

    # obj:CollisionObject = list(objs.items())[0][1]
    # p1 = np.ndarray([obj.pose.position.x,obj.pose.position.y,obj.pose.position.z])
    p1 = np.array(rand_pos)
    log.info("Motion planning to scene object")

    res, target, joint_target = arm_commander.go_position_ikpoints(p1)
    if not res:
        log.info("Motion planning object failed.. trying to plan further away")
        dx = 0.1
        for i in range(iterations):
            p2 = p1 * (1 - dx*i)
            log.info("scaled point by %f" %(1-dx*i))
            res, target, joint_target = arm_commander.go_position_ikpoints(p2)
            if res:
                break
    assert res , "motion planning failed"

    log.info("Boolean result: %s" % res)
    log.info("Target type: %s" % type(target))
    assert_ik_result(arm_commander.get_end_effector_pose(),target,PLANNING_TOLERANCE)

def test_go_rand_pose_loop():
    success_counter = 0
    for _ in range(iterations):
        res, target = arm_commander.go_pose()
        compare_res, msg = compare_ik_result(arm_commander.get_end_effector_pose(),target,PLANNING_TOLERANCE)
        if res and compare_res:
            success_counter+=1
    
    assert success_counter/10 >= SUCCESS_RATE_TOLERANCE, "Motion planning success rate is too low. %f" %(success_counter/10)

def test_go_rand_ikpoint_loop():
    success_counter = 0
    for _ in range(iterations):
        res, target, joint_target = arm_commander.go_position_ikpoints()
        compare_res, msg = compare_ik_result(arm_commander.get_end_effector_pose(),target,PLANNING_TOLERANCE)
        if res and compare_res:
            success_counter+=1
    
    assert success_counter/10 >= SUCCESS_RATE_TOLERANCE, "Motion planning success rate is too low. %f" %(success_counter/10)

def test_avg_go_ikpoint_dist():
    d = 0
    for _ in range(iterations):
        res, target, joint_target = arm_commander.go_position_ikpoints()
        if res:
            d+=dist(target,arm_commander.get_end_effector_pose())
    d_avg = d/iterations

    assert d_avg <= PLANNING_TOLERANCE, "Avg ik point distance too far: %s cm" %(d_avg*100)
        


    