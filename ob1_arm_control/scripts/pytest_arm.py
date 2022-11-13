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
from geometry_msgs.msg import PoseStamped, Pose, Vector3Stamped, Quaternion, Point
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

test_log = logging.getLogger(__name__)

arm_commander = ArmCommander(sample_time_out=0.1,goal_tolerance=0.02)
test_log.info("Clearing scene..")
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
        return True, d, "Motion planning tolerance success. %f cm" % (d*100)
    else:
        return False, d, "Distance from target is too far, %f cm" % (d*100)

def test_go_rand_pose():
    test_log.info("Starting go_rand_pose test")
    test_log.info("Going to random pose...")
    res, target = arm_commander.go_pose()
    assert res , "motion planning failed"
    test_log.info("Boolean result: %s" % res)
    test_log.info("Target type: %s" % type(target))
    assert_ik_result(arm_commander.get_end_effector_pose(),target,PLANNING_TOLERANCE)

def test_go_rand_position():
    test_log.info("Starting go_rand_position test")
    test_log.info("Going to random position...")
    res, target = arm_commander.go_position()
    assert res , "motion planning failed"
    test_log.info("Boolean result: %s" % res)
    test_log.info("Target type: %s" % type(target))
    assert_ik_result(arm_commander.get_end_effector_pose(),target,PLANNING_TOLERANCE)

def test_go_rand_joint_target():
    test_log.info("Starting go_rand_joint_target test")
    test_log.info("Going to random joint target...")
    res, target = arm_commander.go_joint()
    assert res , "motion planning failed"
    test_log.info("Boolean result: %s" % res)
    test_log.info("Target type: %s" % type(target))
    assert_ik_result(arm_commander.arm_mvgroup.get_current_joint_values(),target,PLANNING_TOLERANCE)

def test_go_rand_ikpoint():
    test_log.info("Starting go_rand_ikpoint test")
    test_log.info("Generating random point and matching nearest ikpoint...")
    res, target, joint_target = arm_commander.go_position_ikpoints()
    assert res , "motion planning failed"
    test_log.info("Boolean result: %s" % res)
    test_log.info("Target type: %s" % type(target))
    assert_ik_result(arm_commander.get_end_effector_pose(),target,PLANNING_TOLERANCE)

def test_go_reachable_scene_object_1():
    test_log.info("Starting go_reachable_scene_object_1 test")

    test_log.info("Clearing scene..")
    arm_commander.scene.clear()

    test_log.info("Generating and placing scene object..")
    p = PoseStamped()
    p.header.frame_id = arm_commander.robot.get_planning_frame()
    p.pose.position = arm_commander.arm_mvgroup.get_random_pose().pose.position
    p.pose.orientation.w = 1
    arm_commander.scene.add_sphere("sphere",p,0.1)

    time.sleep(5)

    test_log.info("Detecting scene objects..")
    objs = arm_commander.scene.get_objects()
    assert len(objs) > 0, "Could not find any scene objects"
    obj:CollisionObject = list(objs.items())[0][1]
    pos = [obj.pose.position.x,obj.pose.position.y,obj.pose.position.z]
    # pos = rand_pos

    test_log.info("Motion planning to scene object")
    res, target, joint_target = arm_commander.go_position_ikpoints(pos)

    assert res , "motion planning failed"

    test_log.info("Boolean result: %s" % res)
    test_log.info("Target type: %s" % type(target))
    assert_ik_result(arm_commander.get_end_effector_pose(),target,PLANNING_TOLERANCE)

@pytest.mark.parametrize("iterations", [1,5,10,15,20])
def test_go_reachable_scene_object_2(iterations):
    test_log.info("Starting go_rand_scene_object_3 test")

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
        
        compare_res,d, msg = compare_ik_result(arm_commander.get_end_effector_pose(),target,PLANNING_TOLERANCE)
        test_log.info(compare_res)
        if res and compare_res:
            success_counter+=1
    
    test_log.info("Success rate: %f%%" %(success_counter/iterations*100))
    assert success_counter/iterations >= SUCCESS_RATE_TOLERANCE

@pytest.mark.parametrize("iterations", [1,5,10,15,20])
def test_go_rand_scene_object_2(iterations):
    test_log.info("Starting go_rand_scene_object_2 test")

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
        
        compare_res,d, msg = compare_ik_result(arm_commander.get_end_effector_pose(),target,PLANNING_TOLERANCE)
        test_log.info(compare_res)
        if res and compare_res:
            success_counter+=1
    
    test_log.info("Success rate: %f%%" %(success_counter/iterations*100))
    assert success_counter/iterations >= SUCCESS_RATE_TOLERANCE

@pytest.mark.parametrize("claw_search_tolerance", [0.05,0.1])
@pytest.mark.parametrize("orientation_search_tolerance", [0.1,0.25,0.5,1])
@pytest.mark.parametrize("iterations", [5])
@pytest.mark.parametrize("sphere_radius", [0.03])
def test_go_reachable_scene_object_smart(claw_search_tolerance,orientation_search_tolerance,iterations,sphere_radius):
    test_log.info("Starting go_reachable_scene_object_smart test")

    success_counter = 0
    dist_total = 0
    for _ in range(iterations):

        arm_commander.scene.clear()

        p = PoseStamped()
        p.header.frame_id = arm_commander.robot.get_planning_frame()
        p.pose.position = arm_commander.arm_mvgroup.get_random_pose().pose.position
        p.pose.orientation.w = 1
        arm_commander.scene.add_sphere("sphere",p,sphere_radius)

        time.sleep(2)

        objs = arm_commander.scene.get_objects()
        if len(objs) < 1:
            test_log.warning("Could not find any scene objects")
            continue
        obj:CollisionObject = list(objs.items())[0][1]

        res, obj_pose, joint_target = arm_commander.go_scene_object(obj, claw_search_tolerance,orientation_search_tolerance )

        compare_res,d, msg = compare_ik_result(arm_commander.get_end_effector_pose(), obj_pose, PLANNING_TOLERANCE)
        test_log.info(msg)

        if res and compare_res:
            success_counter+=1
        dist_total+=dist(arm_commander.get_end_effector_pose(),obj_pose)
        
    test_log.info("Motion planning success rate: %f%%" %(success_counter/iterations*100))
    test_log.info("Motion planning avg tolerance: %f cm " %(dist_total/iterations*100))
    assert success_counter/iterations >= SUCCESS_RATE_TOLERANCE, "Motion planning success rate is too low. %f" %(success_counter/iterations)


@pytest.mark.parametrize("sphere_radius", [0.03])
@pytest.mark.parametrize("scale_trans", [
    (0.1,[0,-0.05,0]),
    (0.2,[0,-0.05,0]),
    (0.3,[0,-0.05,0]),
    (0.4,[0,-0.05,0]),
    (0.5,[0,-0.05,0]),
    (0.1,[0,-0.05,0]),
    (0.1,[0.05,-0.05,0]),
    (0.1,[-0.05,-0.05,0])
    ])
def test_viz_target(scale_trans,sphere_radius):
    arm_commander.scene.remove_world_object()

    def get_pose_viz(scale:float,translate:list):
        trans,rot = arm_commander.tf_listener.lookupTransform('/'+arm_commander.GRIPPER_LCLAW_LINK_NAME, '/'+arm_commander.GRIPPER_RCLAW_LINK_NAME, rospy.Time(0))
        trans = np.array([trans[0],trans[1],trans[2]]) #relative offset from rclaw to lcalw
        trans=trans*scale + np.array(translate) #translate to between claws and upwards in frame
        pt_mid = Point(trans[0],trans[1],trans[2])
        q_mid = Quaternion(rot[0],rot[1],rot[2],rot[3])
        pose_mid = Pose(pt_mid,q_mid)
        posestamped_mid = PoseStamped()
        posestamped_mid.pose = pose_mid
        posestamped_mid.header.frame_id = arm_commander.GRIPPER_RCLAW_LINK_NAME
        return arm_commander.tf_listener.transformPose('/world',posestamped_mid)

    scale, trans = scale_trans
    pose_viz = get_pose_viz(scale,trans)
    arm_commander.scene.add_sphere("viz_sphere",pose_viz,sphere_radius)
    test_log.info("Trans: %s, Scale: %s" %(trans,scale))
    time.sleep(15)
    arm_commander.scene.remove_world_object("viz_sphere")

@pytest.mark.parametrize("iterations", [10])
def test_go_rand_pose_loop(iterations):
    success_counter = 0
    for _ in range(iterations):
        res, target = arm_commander.go_pose()
        compare_res,d, msg = compare_ik_result(arm_commander.get_end_effector_pose(),target,PLANNING_TOLERANCE)
        test_log.info("Planning Result: %s, Tolerance %s"%(res,d*100))
        if res and compare_res:
            success_counter+=1
    
    test_log.info("Motion planning success rate: %f%%" %(success_counter/iterations*100))
    assert success_counter/iterations >= SUCCESS_RATE_TOLERANCE, "Motion planning success rate is too low. %f" %(success_counter/iterations)

@pytest.mark.parametrize("iterations", [1,10,25])
def test_go_rand_pose_loop_kinpy(iterations):
    success_counter = 0
    for _ in range(iterations):
        res, target = arm_commander.go_pose_kinpy()
        time.sleep(5)
        compare_res,d, msg = compare_ik_result(arm_commander.get_end_effector_pose(),target,PLANNING_TOLERANCE)
        test_log.info("Planning Result: %s, Tolerance %s"%(res,d*100))
        if res and compare_res:
            success_counter+=1
    
    test_log.info("Motion planning success rate: %f%%" %(success_counter/iterations*100))
    assert success_counter/iterations >= SUCCESS_RATE_TOLERANCE, "Motion planning success rate is too low. %f" %(success_counter/iterations)

@pytest.mark.parametrize("iterations", [1,10,25])
def test_go_rand_ikpoint_loop(iterations):
    success_counter = 0
    for _ in range(iterations):
        res, target, joint_target = arm_commander.go_position_ikpoints()
        compare_res,d, msg = compare_ik_result(arm_commander.get_end_effector_pose(),target,PLANNING_TOLERANCE)
        test_log.info("Planning Result: %s, Tolerance %s"%(res,d*100))
        if res and compare_res:
            success_counter+=1
    
    test_log.info("Motion planning success rate: %f%%" %(success_counter/iterations*100))
    assert success_counter/iterations >= SUCCESS_RATE_TOLERANCE, "Motion planning success rate is too low. %f" %(success_counter/iterations)

@pytest.mark.parametrize("iterations", [1,10,25])
def test_avg_go_ikpoint_dist(iterations):
    d = 0
    for _ in range(iterations):
        res, target, joint_target = arm_commander.go_position_ikpoints()
        if res:
            d+=dist(target,arm_commander.get_end_effector_pose())
    d_avg = d/iterations

    test_log.info("Avg ik point distance: %s cm" %(d_avg*100))
    assert d_avg <= PLANNING_TOLERANCE, "Avg ik point distance too far: %s cm" %(d_avg*100)
        


    