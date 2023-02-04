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
from tf_helpers import broadcast_pose
import kinpy as kp
print(sys.version)

##TEST PARAMS
# rp = rospkg.RosPack()
# PACKAGE_PATH = rp.get_path('ob1_arm_control')
# DATA_FILE_PATH = PACKAGE_PATH+"/data/5k_pose_data.pickle"

PLANNING_TOLERANCE = 0.001 #distance in metres
SUCCESS_RATE_TOLERANCE = 0.80 #percentage of tests that pass

test_log = logging.getLogger(__name__)

arm_commander = ArmCommander(sample_time_out=5,goal_tolerance=0.02,sample_attempts=10)
test_log.info("Clearing scene..")
arm_commander.scene.clear()

def spawn_random_sphere(r):
    """
    @brief spawn random sphere in scene with name 'sphere'
    @param r: float sphere radium in metres
    """
    p = PoseStamped()
    p.header.frame_id = arm_commander.robot.get_planning_frame()
    p.pose.position = arm_commander.arm_mvgroup.get_random_pose().pose.position
    p.pose.orientation.w = 1
    arm_commander.scene.add_sphere("sphere",p,r)

    time.sleep(2)

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
    
###########################################################################3

def test_go_rand_pose_1():
    test_log.info("Starting go_rand_pose test")
    test_log.info("Going to random pose...")
    target = arm_commander.arm_mvgroup.get_random_pose()
    broadcast_pose(arm_commander.tf_broadcaster,target.pose,'target','world')
    res, _ = arm_commander.go_pose(target)
    assert res , "motion planning failed"
    # assert all_close(target, arm_commander.arm_mvgroup.get_current_joint_values()) , "joint target not within tolerance"
    test_log.info("Boolean result: %s" % res)
    test_log.info("Target type: %s" % type(target))
    assert_ik_result(arm_commander.get_end_effector_pose(),target,PLANNING_TOLERANCE)

def test_go_rand_pose_2():
    test_log.info("Starting go_rand_pose test")
    test_log.info("Going to random pose...")
    target = arm_commander.arm_mvgroup.get_random_pose()
    target.pose.position.x = 0.3
    target.pose.position.y = 0
    target.pose.position.z = 0.5
    target.pose.orientation = Quaternion(0,0,0,1)
    broadcast_pose(arm_commander.tf_broadcaster,target.pose,'target','world')
    res, _ = arm_commander.go_pose(target)
    assert res , "motion planning failed"
    # assert all_close(target, arm_commander.arm_mvgroup.get_current_joint_values()) , "joint target not within tolerance"
    test_log.info("Boolean result: %s" % res)
    test_log.info("Target type: %s" % type(target))
    assert_ik_result(arm_commander.get_end_effector_pose(),target,PLANNING_TOLERANCE)

def test_go_position_1():
    target = arm_commander.arm_mvgroup.get_random_pose()
    broadcast_pose(arm_commander.tf_broadcaster,target.pose,'target','world')
    arm_commander.arm_mvgroup.set_position_target(convert_to_list(target.pose.position))
    assert arm_commander.arm_mvgroup.go()

def test_go_rand_position():
    test_log.info("Starting go_rand_position test")
    test_log.info("Going to random position...")
    target = arm_commander.arm_mvgroup.get_random_pose()
    broadcast_pose(arm_commander.tf_broadcaster,target.pose,'target','world')
    res, _ = arm_commander.go_position(target.pose.position)
    assert res , "motion planning failed"
    test_log.info("Boolean result: %s" % res)
    test_log.info("Target type: %s" % type(target))
    assert_ik_result(arm_commander.get_end_effector_pose().pose.position, target.pose.position, PLANNING_TOLERANCE)

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

@pytest.mark.parametrize("iterations", [5,10,25])
@pytest.mark.parametrize("sphere_radius", [0.03, 0.05])
def test_go_reachable_scene_object_pose(iterations,sphere_radius):
    test_log.info("Starting go_reachable_scene_object_pose test")

    success_counter = 0
    dist_total = 0
    for _ in range(iterations):

        arm_commander.scene.clear()
        spawn_random_sphere(sphere_radius)
        objs = arm_commander.scene.get_objects()
        if len(objs) < 1:
            test_log.warning("Could not find any scene objects")
            continue
        obj:CollisionObject = list(objs.items())[0][1]
        broadcast_pose(arm_commander.tf_broadcaster,obj.pose,'sphere','world')

        res, target, _ = arm_commander.go_position_ikpoints(obj.pose.position)

        compare_res,d, msg = compare_ik_result(arm_commander.get_end_effector_pose(), target, PLANNING_TOLERANCE)
        test_log.info(msg)

        if res and compare_res:
            success_counter+=1
        dist_total+=dist(arm_commander.get_end_effector_pose(),target)
        
    test_log.info("Motion planning success rate: %f%%" %(success_counter/iterations*100))
    test_log.info("Motion planning avg tolerance: %f cm " %(dist_total/iterations*100))
    assert success_counter/iterations >= SUCCESS_RATE_TOLERANCE, "Motion planning success rate is too low. %f" %(success_counter/iterations)

@pytest.mark.parametrize("iterations", [5])
@pytest.mark.parametrize("sphere_radius", [0.03])
def test_pick(iterations,sphere_radius):
    test_log.info("Starting pick test")

    success_counter = 0
    for _ in range(iterations):

        arm_commander.scene.clear()
        spawn_random_sphere(sphere_radius)
        objs = arm_commander.scene.get_objects()
        if len(objs) < 1:
            test_log.warning("Could not find any scene objects")
            continue
        obj:CollisionObject = list(objs.items())[0][1]
        broadcast_pose(arm_commander.tf_broadcaster,obj.pose,'sphere','world')

        res = arm_commander.pick_object_ikpoints(obj)
        test_log.info(res)

        if res :
            success_counter+=1

    test_log.info("Motion planning success rate: %f%%" %(success_counter/iterations*100))
    assert success_counter/iterations >= SUCCESS_RATE_TOLERANCE, "Motion planning success rate is too low. %f" %(success_counter/iterations)

@pytest.mark.parametrize("sphere_radius", [0.03])
def test_pick_and_move(sphere_radius):
    test_log.info("Starting pick and move test")

    arm_commander.scene.clear()
    spawn_random_sphere(sphere_radius)
    objs = arm_commander.scene.get_objects()
    assert len(objs) >= 0, 'could not find any scene objects'
    obj:CollisionObject = list(objs.items())[0][1]
    broadcast_pose(arm_commander.tf_broadcaster,obj.pose,'sphere','world')

    assert arm_commander.pick_object(obj), 'failed to pick object'
    
    assert arm_commander.go_position()[0], 'failed to move arm after attaching object'

    assert arm_commander.detach_object(obj), 'failed to detach object'

    assert arm_commander.open_gripper(), 'failed to open gripper after detaching object'

@pytest.mark.parametrize("sphere_radius", [0.03])
def test_pick_and_move_2(sphere_radius):
    test_log.info("Starting pick and move test")

    arm_commander.scene.clear()
    
    p = PoseStamped()
    p.header.frame_id = arm_commander.robot.get_planning_frame()
    p.pose.position = arm_commander.arm_mvgroup.get_random_pose().pose.position
    p.pose.position.x = 0.3
    p.pose.position.y = 0
    p.pose.position.z = 0.2
    p.pose.orientation.w = 1
    arm_commander.scene.add_sphere("sphere",p,sphere_radius)

    objs = arm_commander.scene.get_objects()
    assert len(objs) >= 0, 'could not find any scene objects'
    obj:CollisionObject = list(objs.items())[0][1]
    broadcast_pose(arm_commander.tf_broadcaster,obj.pose,'sphere','world')

    assert arm_commander.pick_object(obj), 'failed to pick object'
    
    assert arm_commander.go_position()[0], 'failed to move arm after attaching object'

    assert arm_commander.detach_object(obj), 'failed to detach object'

    assert arm_commander.open_gripper(), 'failed to open gripper after detaching object'

def test_pick_cylinder_and_move():
    test_log.info("Starting pick and move test")

    arm_commander.scene.clear()

    p = PoseStamped()
    p.header.frame_id = arm_commander.robot.get_planning_frame()
    p.pose.position = arm_commander.arm_mvgroup.get_random_pose().pose.position
    p.pose.orientation.w = 1
    arm_commander.scene.add_cylinder('cylinder', p, height=0.15, radius=0.03)

    time.sleep(2)

    objs = arm_commander.scene.get_objects()
    assert len(objs) >= 0, 'could not find any scene objects'
    obj:CollisionObject = list(objs.items())[0][1]
    broadcast_pose(arm_commander.tf_broadcaster,obj.pose,'cylinder','world')

    assert arm_commander.pick_object_ikpoints(obj), 'failed to pick object'
    
    assert arm_commander.go_position_ikpoints()[0], 'failed to move arm after attaching object'

    assert arm_commander.detach_object(obj), 'failed to detach object'

    assert arm_commander.open_gripper(), 'failed to open gripper after detaching object'

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
def _test_viz_target(scale_trans,sphere_radius):
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
def ntest_go_rand_pose_loop(iterations):
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
def test_go_pose_kinpy(iterations):
    success_counter = 0
    for _ in range(iterations):
        pose_goal = arm_commander.arm_mvgroup.get_random_pose().pose
        broadcast_pose(arm_commander.tf_broadcaster, pose_goal, 'target', 'world')
        rot = np.array(convert_to_list(pose_goal.orientation))
        pos = np.array(convert_to_list(pose_goal.position))
        pose_tf =  kp.Transform(rot,pos)
        ik_sol_joints = convert_to_list(arm_commander.kinpy_arm.inverse_kinematics(pose_tf))
        res = False
        if arm_commander._check_joint_limits(ik_sol_joints):
            res, _ = arm_commander.go_joint(ik_sol_joints)
        compare_res,d, msg = compare_ik_result(arm_commander.get_end_effector_pose(),pose_goal,PLANNING_TOLERANCE)
        test_log.info("Planning Result: %s, Tolerance %s"%(res,d*100))
        if res and compare_res:
            success_counter+=1
    
    test_log.info("Motion planning success rate: %f%%" %(success_counter/iterations*100))
    assert success_counter/iterations >= SUCCESS_RATE_TOLERANCE, "Motion planning success rate is too low. %f" %(success_counter/iterations)

@pytest.mark.parametrize("iterations", [1,10,25])
def ntest_go_rand_ikpoint_loop(iterations):
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
def ntest_avg_go_ikpoint_dist(iterations):
    d = 0
    for _ in range(iterations):
        res, target, joint_target = arm_commander.go_position_ikpoints()
        if res:
            d+=dist(target,arm_commander.get_end_effector_pose())
    d_avg = d/iterations

    test_log.info("Avg ik point distance: %s cm" %(d_avg*100))
    assert d_avg <= PLANNING_TOLERANCE, "Avg ik point distance too far: %s cm" %(d_avg*100)
        


    