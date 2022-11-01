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
from geometry_msgs.msg import PoseStamped, Pose, Vector3Stamped, Quaternion
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

PLANNING_TOLERANCE = 0.1 #distance in metres
SUCCESS_RATE_TOLERANCE = 0.80 #percentage of tests that pass

log = logging.getLogger(__name__)

arm_commander = ArmCommander(sample_time_out=5,goal_tolerance=0.1)
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
            return False, "transform frame mismatch. Object pose is in %s frame" % target.header.frame_id
    d = dist(current_state,target)
    if d <= tolerance: 
        return True, "Motion planning tolerance success. %f cm" % (d*100)
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

def test_go_reachable_scene_object_1():
    log.info("Starting go_reachable_scene_object_1 test")

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

    time.sleep(5)

    log.info("Detecting scene objects..")
    objs = arm_commander.scene.get_objects()
    assert len(objs) > 0, "Could not find any scene objects"
    obj:CollisionObject = list(objs.items())[0][1]
    pos = [obj.pose.position.x,obj.pose.position.y,obj.pose.position.z]
    # pos = rand_pos

    log.info("Motion planning to scene object")
    res, target, joint_target = arm_commander.go_position_ikpoints(pos)

    assert res , "motion planning failed"

    log.info("Boolean result: %s" % res)
    log.info("Target type: %s" % type(target))
    assert_ik_result(arm_commander.get_end_effector_pose(),target,PLANNING_TOLERANCE)

@pytest.mark.parametrize("iterations", [1,5,10,15,20])
def test_go_reachable_scene_object_2(iterations):
    log.info("Starting go_rand_scene_object_3 test")

    success_counter = 0
    for _ in range(iterations):

        arm_commander.scene.clear()

        p = PoseStamped()
        p.header.frame_id = arm_commander.robot.get_planning_frame()
        rand_pos = arm_commander.ikpoints.get_rand_point()
        p.pose.position.x = rand_pos[0]
        p.pose.position.y = rand_pos[1]
        p.pose.position.z = rand_pos[2]
        p.pose.orientation.w = 1
        arm_commander.scene.add_sphere("sphere",p,0.05)

        p1 = np.array(rand_pos)

        res, target, joint_target = arm_commander.go_position_ikpoints(p1)
        if not res:
            dx = 0.1
            for i in range(iterations):
                p2 = p1 * (1 - dx*i)
                res, target, joint_target = arm_commander.go_position_ikpoints(p2)
                if res:
                    break
        
        compare_res, msg = compare_ik_result(arm_commander.get_end_effector_pose(),target,PLANNING_TOLERANCE)
        log.info(compare_res)
        if res and compare_res:
            success_counter+=1
    
    log.info("Success rate: %f%%" %(success_counter/iterations*100))
    assert success_counter/iterations >= SUCCESS_RATE_TOLERANCE

@pytest.mark.parametrize("iterations", [1,5,10,15,20])
def test_go_rand_scene_object_2(iterations):
    log.info("Starting go_rand_scene_object_2 test")

    success_counter = 0
    for _ in range(iterations):

        arm_commander.scene.clear()

        p = arm_commander.arm_mvgroup.get_random_pose()
        p.header.frame_id = arm_commander.robot.get_planning_frame()
        p.pose.orientation = Quaternion()
        arm_commander.scene.add_sphere("sphere",p,0.05)

        p1 = np.array(convert_to_list(p.pose))

        res, target, joint_target = arm_commander.go_position_ikpoints(p1)
        if not res:
            dx = 0.1
            for i in range(iterations):
                p2 = p1 * (1 - dx*i)
                res, target, joint_target = arm_commander.go_position_ikpoints(p2)
                if res:
                    break
        
        compare_res, msg = compare_ik_result(arm_commander.get_end_effector_pose(),target,PLANNING_TOLERANCE)
        log.info(compare_res)
        if res and compare_res:
            success_counter+=1
    
    log.info("Success rate: %f%%" %(success_counter/iterations*100))
    assert success_counter/iterations >= SUCCESS_RATE_TOLERANCE

@pytest.mark.parametrize("attempts", [10])
@pytest.mark.parametrize("iterations", [1,10,25])
@pytest.mark.parametrize("sphere_radius", [0.03])
def test_go_reachable_scene_object_smart(attempts,iterations,sphere_radius):
    log.info("Starting go_reachable_scene_object_smart test")

    success_counter = 0
    dist_total = 0
    for _ in range(iterations):

        arm_commander.scene.clear()

        p = PoseStamped()
        p.header.frame_id = arm_commander.robot.get_planning_frame()
        rand_pos = arm_commander.ikpoints.get_rand_point()
        p.pose.position.x = rand_pos[0]
        p.pose.position.y = rand_pos[1]
        p.pose.position.z = rand_pos[2]
        p.pose.orientation.w = 1
        arm_commander.scene.add_sphere("sphere",p,sphere_radius)

        time.sleep(2)

        objs = arm_commander.scene.get_objects()
        if len(objs) < 1:
            log.warning("Could not find any scene objects")
            continue
        obj:CollisionObject = list(objs.items())[0][1]

        res, obj_pose, joint_target = arm_commander.go_scene_object(obj,attempts)

        compare_res, msg = compare_ik_result(arm_commander.get_end_effector_pose(), obj_pose, PLANNING_TOLERANCE)
        log.info(msg)

        if res and compare_res:
            success_counter+=1
        dist_total+=dist(arm_commander.get_end_effector_pose(),obj_pose)
        
    log.info("Motion planning success rate: %f%%" %(success_counter/iterations*100))
    log.info("Motion planning avg tolerance: %f cm " %(dist_total/iterations*100))
    assert success_counter/iterations >= SUCCESS_RATE_TOLERANCE, "Motion planning success rate is too low. %f" %(success_counter/iterations)

@pytest.mark.parametrize("iterations", [1,10,25])
def test_go_rand_pose_loop(iterations):
    success_counter = 0
    for _ in range(iterations):
        res, target = arm_commander.go_pose()
        compare_res, msg = compare_ik_result(arm_commander.get_end_effector_pose(),target,PLANNING_TOLERANCE)
        if res and compare_res:
            success_counter+=1
    
    log.info("Motion planning success rate: %f%%" %(success_counter/iterations*100))
    assert success_counter/iterations >= SUCCESS_RATE_TOLERANCE, "Motion planning success rate is too low. %f" %(success_counter/iterations)

@pytest.mark.parametrize("iterations", [1,10,25])
def test_go_rand_ikpoint_loop(iterations):
    success_counter = 0
    for _ in range(iterations):
        res, target, joint_target = arm_commander.go_position_ikpoints()
        compare_res, msg = compare_ik_result(arm_commander.get_end_effector_pose(),target,PLANNING_TOLERANCE)
        if res and compare_res:
            success_counter+=1
    
    log.info("Motion planning success rate: %f%%" %(success_counter/iterations*100))
    assert success_counter/iterations >= SUCCESS_RATE_TOLERANCE, "Motion planning success rate is too low. %f" %(success_counter/iterations)

@pytest.mark.parametrize("iterations", [1,10,25])
def test_avg_go_ikpoint_dist(iterations):
    d = 0
    for _ in range(iterations):
        res, target, joint_target = arm_commander.go_position_ikpoints()
        if res:
            d+=dist(target,arm_commander.get_end_effector_pose())
    d_avg = d/iterations

    log.info("Avg ik point distance: %s cm" %(d_avg*100))
    assert d_avg <= PLANNING_TOLERANCE, "Avg ik point distance too far: %s cm" %(d_avg*100)
        


    