#! /usr/bin/env python
from __future__ import print_function
from operator import itemgetter
import pickle
import time
from arm_commander import ArmCommander
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
from tf_helpers import broadcast_pose, broadcast_point, convert_to_list, QuaternionComparators
import kinpy as kp
from test_helpers import *
import test_helpers
import pyquaternion as pyq
import random
from ob1_arm_control.srv import IKPointsServiceRequest
from ikpoints_service import ikpoints_service_client
from relaxed_ik.srv import RelaxedIKService, RelaxedIKServiceRequest
from relaxed_ik_helpers import relaxedik_service_client
from functools import cmp_to_key
print(sys.version)

GOAL_TOLERANCE = 0.01 #elementwise distance and angles for orientation
JOINT_TOLERANCE = 0.001 #element wise angles
DIST_TOLERANCE = 0.01 #distance in m
SUCCESS_RATE_TOLERANCE = 0.85 #percentage of tests that pass

log = logging.getLogger(__name__)
arm_commander = ArmCommander(sample_timeout=0.1)

def clear_scene():
    arm_commander.scene.clear()

@pytest.fixture
def setup_teardown():
    clear_scene()
    arm_commander.open_gripper()
    arm_commander.go_joint([0,0,0,0,0])
    yield
    clear_scene()
    arm_commander.open_gripper()
    arm_commander.go_joint([0,0,0,0,0])

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

##############################################################################################

def test_pyquaternion():
    rosQ = Quaternion(random.uniform(0,1),random.uniform(0,1),random.uniform(0,1),random.uniform(0,1))
    pyQ = pyq.Quaternion(rosQ.w,rosQ.x,rosQ.y,rosQ.z)
    assert rosQ.x == pyQ.x
    assert rosQ.y == pyQ.y
    assert rosQ.z == pyQ.z
    assert rosQ.w == pyQ.w

@pytest.mark.parametrize("iterations", [5,10])
def test_quaternion_comparators(setup_teardown, iterations):
    avg_accuracies = []
    req = IKPointsServiceRequest()
    req.request = 'get up to dist targets'
    req.distance = DIST_TOLERANCE

    for comparator_id in range(len(QuaternionComparators.comparators())):
        total = 0
        for _ in range(iterations):
            target = arm_commander.arm_mvgroup.get_random_pose()
            broadcast_pose(arm_commander.tf_broadcaster,target.pose,'target','world')

            req.pose = target.pose
            pose_targets , joint_targets, _ = ikpoints_service_client(req)

            indices = arm_commander.get_indices_optimal_poses(target.pose, pose_targets, 1, comparator_id=comparator_id)
            
            res = False
            for i in indices:
                joints = joint_targets[i]
                res, _ = arm_commander.go_joint(joints)
                if res:
                    break
            total += avg_all_close(pose_to_list(target.pose), pose_to_list(arm_commander.get_end_effector_pose().pose))
        avg_accuracies.append(total/iterations)
    
    log.info(avg_accuracies)
    for comparator_id in range(len(QuaternionComparators.comparators())):
        assert avg_accuracies[comparator_id] <= GOAL_TOLERANCE, "comparator %d failed tolerance check"%comparator_id
    
def test_quaternion_doublesort(setup_teardown):
    avg_accuracies = []
    req = IKPointsServiceRequest()
    req.request = 'get up to dist targets'
    req.distance = DIST_TOLERANCE

    target = arm_commander.arm_mvgroup.get_random_pose()
    broadcast_pose(arm_commander.tf_broadcaster,target.pose,'target','world')

    req.pose = target.pose
    pose_targets , joint_targets, _ = ikpoints_service_client(req)

    indices = arm_commander.get_indices_optimal_poses(target.pose, pose_targets, 1000, comparator_id=2)
    indices = arm_commander.get_indices_optimal_poses(target.pose, [pose_targets[i] for i in indices][:10], 1, comparator_id=3)
    
    res = False
    for i in indices:
        joints = joint_targets[i]
        res, _ = arm_commander.go_joint(joints)
        if res:
            break
    assert res, "motion planning failed"
    assert avg_all_close(pose_to_list(target.pose), pose_to_list(arm_commander.get_end_effector_pose().pose)) <= GOAL_TOLERANCE, \
        "average tolerance is not within specified goal tolerance"

def test_go_rand_pose_dist(setup_teardown):
    target = arm_commander.arm_mvgroup.get_random_pose()
    broadcast_pose(arm_commander.tf_broadcaster,target.pose,'target','world')
    res, _ = arm_commander.go_pose(target)
    assert res , "motion planning failed"
    assert_dist(arm_commander.get_end_effector_pose(),target,DIST_TOLERANCE)

def test_go_rand_pose_tolerance(setup_teardown):
    target = arm_commander.arm_mvgroup.get_random_pose()
    broadcast_pose(arm_commander.tf_broadcaster,target.pose,'target','world')
    res, _ = arm_commander.go_pose(target)
    assert res , "motion planning failed"
    assert all_close(target, arm_commander.get_end_effector_pose(), GOAL_TOLERANCE)

def test_go_rand_pose_ikpoint(setup_teardown):
    target = arm_commander.arm_mvgroup.get_random_pose()
    broadcast_pose(arm_commander.tf_broadcaster,target.pose,'target','world')
    res, _, joints = arm_commander.go_pose_ikpoints(target)
    assert res , "motion planning failed"
    assert all_close(joints, arm_commander.arm_mvgroup.get_current_joint_values(), JOINT_TOLERANCE) , "joint target not within tolerance"
    assert_dist(arm_commander.get_end_effector_pose(),target,DIST_TOLERANCE)
    assert all_close(target.pose, arm_commander.get_end_effector_pose().pose, GOAL_TOLERANCE)

def test_go_rand_pose_relaxedik(setup_teardown):
    target = arm_commander.arm_mvgroup.get_random_pose()
    broadcast_pose(arm_commander.tf_broadcaster,target.pose,'target','world')
    res, _, joints = arm_commander.go_pose_relaxedik(target)
    assert res , "motion planning failed"
    assert all_close(joints, arm_commander.arm_mvgroup.get_current_joint_values(), JOINT_TOLERANCE) , "joint target not within tolerance"
    assert_dist(arm_commander.get_end_effector_pose(),target, DIST_TOLERANCE)

def test_go_rand_position(setup_teardown):
    target = arm_commander.arm_mvgroup.get_random_pose()
    broadcast_pose(arm_commander.tf_broadcaster,target.pose,'target','world')
    res, _ = arm_commander.go_position(target)
    assert res , "motion planning failed"
    assert_dist(convert_to_list(arm_commander.get_end_effector_pose().pose.position), convert_to_list(target.pose.position), DIST_TOLERANCE)

def test_go_rand_joint_target(setup_teardown):
    res, target = arm_commander.go_joint()
    assert res , "motion planning failed"
    assert all_close(target,arm_commander.arm_mvgroup.get_current_joint_values(),JOINT_TOLERANCE)

def test_go_rand_ikpoint(setup_teardown):
    target = arm_commander.arm_mvgroup.get_random_pose()
    broadcast_pose(arm_commander.tf_broadcaster,target.pose,'target','world')
    res, _, joint_target = arm_commander.go_position_ikpoints(target)
    assert res , "motion planning failed"
    assert all_close(joint_target, arm_commander.arm_mvgroup.get_current_joint_values(), JOINT_TOLERANCE) , "joint target not within tolerance"
    assert_dist(convert_to_list(arm_commander.get_end_effector_pose().pose.position),convert_to_list(target.pose.position), DIST_TOLERANCE)

def test_go_pose_kinpy(setup_teardown):

    rp = rospkg.RosPack()
    urdf_path = rp.get_path('ob1_arm_description') + "/urdf/main.urdf"
    kinpy_arm = kp.build_serial_chain_from_urdf(
        open(urdf_path).read(),
        root_link_name="ob1_arm_base_link",
        end_link_name="ob1_arm_eef_link"
    )
    pose_goal = arm_commander.arm_mvgroup.get_random_pose().pose
    broadcast_pose(arm_commander.tf_broadcaster, pose_goal, 'target', 'world')
    p = convert_to_list(pose_goal)
    rot = np.array(p[3:7])
    pos = np.array(p[:3])
    pose_tf =  kp.Transform(rot,pos)
    ik_sol_joints = convert_to_list(kinpy_arm.inverse_kinematics(pose_tf))
    assert arm_commander._check_joint_limits(ik_sol_joints), "joints exceed physical limits"
    res, _ = arm_commander.go_joint(ik_sol_joints)
    assert res, "motion planning failed"
    assert all_close(ik_sol_joints, arm_commander.arm_mvgroup.get_current_joint_values(), JOINT_TOLERANCE) , "joint target not within tolerance"
    assert_dist(pose_goal, arm_commander.get_end_effector_pose(), DIST_TOLERANCE)

# @pytest.mark.parametrize("claw_search_tolerance", [0.05,0.1])
# @pytest.mark.parametrize("orientation_search_tolerance", [0.1,0.25,0.5,1])
# @pytest.mark.parametrize("iterations", [5])
# @pytest.mark.parametrize("sphere_radius", [0.03])
# def test_go_reachable_scene_object_smart(claw_search_tolerance,orientation_search_tolerance,iterations,sphere_radius):
#     log.info("Starting go_reachable_scene_object_smart test")

#     success_counter = 0
#     dist_total = 0
#     for _ in range(iterations):

#         arm_commander.scene.clear()

#         p = PoseStamped()
#         p.header.frame_id = arm_commander.robot.get_planning_frame()
#         p.pose.position = arm_commander.arm_mvgroup.get_random_pose().pose.position
#         p.pose.orientation.w = 1
#         arm_commander.scene.add_sphere("sphere",p,sphere_radius)

#         time.sleep(2)

#         objs = arm_commander.scene.get_objects()
#         if len(objs) < 1:
#             log.warning("Could not find any scene objects")
#             continue
#         obj:CollisionObject = list(objs.items())[0][1]

#         res, obj_pose, joint_target = arm_commander.go_scene_object(obj, claw_search_tolerance,orientation_search_tolerance )

#         compare_res,d, msg = check_dist(arm_commander.get_end_effector_pose(), obj_pose, GOAL_TOLERANCE)
#         log.info(msg)

#         if res and compare_res:
#             success_counter+=1
#         dist_total+=dist(arm_commander.get_end_effector_pose(),obj_pose)
        
#     log.info("Motion planning success rate: %f%%" %(success_counter/iterations*100))
#     log.info("Motion planning avg tolerance: %f cm " %(dist_total/iterations*100))
#     assert success_counter/iterations >= SUCCESS_RATE_TOLERANCE, "Motion planning success rate is too low. %f" %(success_counter/iterations)

@pytest.mark.parametrize("sphere_radius", [0.03])
def test_go_scene_object(setup_teardown, sphere_radius):
    spawn_random_sphere(sphere_radius)
    objs = arm_commander.scene.get_objects()
    assert len(objs) > 0, "Could not find any scene objects"
    obj:CollisionObject = list(objs.items())[0][1]
    broadcast_pose(arm_commander.tf_broadcaster,obj.pose,'target','world')
    res, target, joint_target = arm_commander.go_position_ikpoints(obj.pose.position)
    assert res , "motion planning failed"
    assert all_close(joint_target, arm_commander.arm_mvgroup.get_current_joint_values(), JOINT_TOLERANCE), "joints not within tolerance"
    assert_dist(arm_commander.get_end_effector_pose().pose.position, obj.pose.position, DIST_TOLERANCE)

@pytest.mark.parametrize("sphere_radius", [0.03])
def test_pick_sphere(setup_teardown, sphere_radius):
    spawn_random_sphere(sphere_radius)
    objs = arm_commander.scene.get_objects()
    assert len(objs) > 0, "Could not find any scene objects"
    obj:CollisionObject = list(objs.items())[0][1]
    broadcast_pose(arm_commander.tf_broadcaster,obj.pose,'target','world')
    assert arm_commander.pick_object(obj)

@pytest.mark.parametrize("sphere_radius", [0.03])
def test_pick_and_move_sphere(setup_teardown, sphere_radius):
    spawn_random_sphere(sphere_radius)
    objs = arm_commander.scene.get_objects()
    assert len(objs) >= 0, 'could not find any scene objects'
    obj:CollisionObject = list(objs.items())[0][1]
    broadcast_pose(arm_commander.tf_broadcaster,obj.pose,'target','world')

    assert arm_commander.pick_object(obj), 'failed to pick object'
    
    assert arm_commander.go_position_ikpoints()[0], 'failed to move arm after attaching object'

    assert arm_commander.detach_object(obj), 'failed to detach object'

    assert arm_commander.open_gripper(), 'failed to open gripper after detaching object'

@pytest.mark.parametrize("pick_tolerance", [0.01, 0.02, 0.03, 0.04, 0.05])
def test_pick_cylinder_and_move(setup_teardown, pick_tolerance):
    p = PoseStamped()
    p.header.frame_id = arm_commander.robot.get_planning_frame()
    p.pose.position = arm_commander.arm_mvgroup.get_random_pose().pose.position
    p.pose.orientation.w = 1
    arm_commander.scene.add_cylinder('cylinder', p, height=0.15, radius=0.03)

    time.sleep(2)

    obj = arm_commander.get_object('cylinder')
    assert obj!=None, 'could not find any scene objects'
    broadcast_pose(arm_commander.tf_broadcaster,obj.pose,'target','world')

    assert arm_commander.pick_object(obj, attempts=1000, pick_tolerance=pick_tolerance), 'failed to pick object'
    
    assert arm_commander.go_position_ikpoints()[0], 'failed to move arm after attaching object'

    assert arm_commander.detach_object(obj), 'failed to detach object'

    assert arm_commander.open_gripper(), 'failed to open gripper after detaching object'

@pytest.mark.parametrize("pick_tolerance", [0.01])
@pytest.mark.parametrize("place_tolerance", [0.05])
@pytest.mark.parametrize("attempts", [10])
def test_pick_and_place_cylinder(setup_teardown,attempts, pick_tolerance, place_tolerance):
    p = PoseStamped()
    p.header.frame_id = arm_commander.robot.get_planning_frame()
    p.pose.position = arm_commander.arm_mvgroup.get_random_pose().pose.position
    p.pose.orientation.w = 1
    arm_commander.scene.add_cylinder('cylinder', p, height=0.15, radius=0.03)

    time.sleep(2)

    obj = arm_commander.get_object('cylinder')
    assert obj!=None, 'could not find any scene objects'
    broadcast_pose(arm_commander.tf_broadcaster,obj.pose,'target','world')

    assert arm_commander.pick_object(obj, attempts=attempts, pick_tolerance=pick_tolerance), 'failed to pick object'

    #pick random place location
    place_posestamped = arm_commander.arm_mvgroup.get_random_pose()
    place_posestamped.pose.orientation = Quaternion(0,0,0,1) #default to upright
    broadcast_pose(arm_commander.tf_broadcaster,place_posestamped.pose,'target','world')

    assert arm_commander.place_object(obj, place_posestamped, attempts=attempts, place_tolerance=place_tolerance), 'failed to place object'

# @pytest.mark.parametrize("sphere_radius", [0.03])
# @pytest.mark.parametrize("scale_trans", [
#     (0.1,[0,-0.05,0]),
#     (0.2,[0,-0.05,0]),
#     (0.3,[0,-0.05,0]),
#     (0.4,[0,-0.05,0]),
#     (0.5,[0,-0.05,0]),
#     (0.1,[0,-0.05,0]),
#     (0.1,[0.05,-0.05,0]),
#     (0.1,[-0.05,-0.05,0])
#     ])
# def _test_viz_target(scale_trans,sphere_radius):
#     arm_commander.scene.remove_world_object()

#     def get_pose_viz(scale:float,translate:list):
#         trans,rot = arm_commander.tf_listener.lookupTransform('/'+arm_commander.GRIPPER_LCLAW_LINK_NAME, '/'+arm_commander.GRIPPER_RCLAW_LINK_NAME, rospy.Time(0))
#         trans = np.array([trans[0],trans[1],trans[2]]) #relative offset from rclaw to lcalw
#         trans=trans*scale + np.array(translate) #translate to between claws and upwards in frame
#         pt_mid = Point(trans[0],trans[1],trans[2])
#         q_mid = Quaternion(rot[0],rot[1],rot[2],rot[3])
#         pose_mid = Pose(pt_mid,q_mid)
#         posestamped_mid = PoseStamped()
#         posestamped_mid.pose = pose_mid
#         posestamped_mid.header.frame_id = arm_commander.GRIPPER_RCLAW_LINK_NAME
#         return arm_commander.tf_listener.transformPose('/world',posestamped_mid)

#     scale, trans = scale_trans
#     pose_viz = get_pose_viz(scale,trans)
#     arm_commander.scene.add_sphere("viz_sphere",pose_viz,sphere_radius)
#     log.info("Trans: %s, Scale: %s" %(trans,scale))
#     time.sleep(15)
#     arm_commander.scene.remove_world_object("viz_sphere")

# @pytest.mark.parametrize("iterations", [1,10,25])
# def test_avg_go_ikpoint_dist(iterations):
#     d = 0
#     for _ in range(iterations):
#         res, target, joint_target = arm_commander.go_position_ikpoints()
#         if res:
#             d+=dist(target,arm_commander.get_end_effector_pose())
#     d_avg = d/iterations

#     log.info("Avg ik point distance: %s cm" %(d_avg*100))
#     assert d_avg <= GOAL_TOLERANCE, "Avg ik point distance too far: %s cm" %(d_avg*100)
        


    