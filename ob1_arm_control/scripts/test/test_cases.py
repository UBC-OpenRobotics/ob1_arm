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
from arm_commander.arm_commander import ArmCommander
from moveit_commander.conversions import pose_to_list
import numpy as np
from geometry_msgs.msg import PoseStamped, Quaternion
from moveit_commander.planning_scene_interface import CollisionObject
import rospkg
import pytest
from util.tf_helpers import broadcast_pose, broadcast_point, convert_to_list, QuaternionComparators, point_to_list, pose_to_mat
from test_helpers import *
import pyquaternion as pyq
import random
import kinpy as kp
from ob1_arm_control.srv import IKPointsServiceRequest
from arm_commander.ikpoints_service import ikpoints_service_client
from copy import deepcopy

###################################################
#                    TEST PARAMETERS
###################################################
GOAL_TOLERANCE = 0.01 #elementwise distance and angles for orientation
JOINT_TOLERANCE = 0.001 #element wise angles
DIST_TOLERANCE = 0.01 #distance in m
SUCCESS_RATE_TOLERANCE = 0.85 #percentage of tests that pass

###################################################
#                    TEST CASES               
###################################################

################### UNIT TESTS ######################
class UnitTests:
    def test_pyquaternion(self):
        rosQ = Quaternion(random.uniform(0,1),random.uniform(0,1),random.uniform(0,1),random.uniform(0,1))
        pyQ = pyq.Quaternion(rosQ.w,rosQ.x,rosQ.y,rosQ.z)
        assert rosQ.x == pyQ.x
        assert rosQ.y == pyQ.y
        assert rosQ.z == pyQ.z
        assert rosQ.w == pyQ.w

    @pytest.mark.parametrize("iterations", [5,10])
    def test_quaternion_comparators(self, arm_commander, log, setup_teardown, iterations):
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
        
    def test_quaternion_doublesort(self, arm_commander, setup_teardown):
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

    def test_allclose_array(self, arm_commander):
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
        res,_ = arm_commander.go_joint(target)
        assert res , "motion planning failed"
        assert all_close(target, arm_commander.arm_mvgroup.get_current_joint_values(), JOINT_TOLERANCE)

    def test_go_position_dist(self, arm_commander, log, setup_teardown):
        target = arm_commander.arm_mvgroup.get_random_pose().pose.position
        broadcast_point(arm_commander.tf_broadcaster,target,'target','world')
        res, _ = arm_commander.go_position(target)
        assert res , "motion planning failed"
        assert_dist(arm_commander.get_end_effector_pose().pose.position, target, DIST_TOLERANCE)

    def test_go_position_allclose(self, arm_commander, log, setup_teardown):
        target = arm_commander.arm_mvgroup.get_random_pose().pose.position
        broadcast_point(arm_commander.tf_broadcaster,target,'target','world')
        res, _ = arm_commander.go_position(target)
        assert res , "motion planning failed"
        assert all_close(target, arm_commander.get_end_effector_pose().pose.position, GOAL_TOLERANCE)
    
    def test_go_pose_dist(self, arm_commander, log, setup_teardown):
        target = arm_commander.arm_mvgroup.get_random_pose()
        broadcast_pose(arm_commander.tf_broadcaster,target.pose,'target','world')
        res, _ = arm_commander.go_pose(target)
        assert res , "motion planning failed"
        assert_dist(arm_commander.get_end_effector_pose(), target, DIST_TOLERANCE)

    def test_go_pose_allclose(self, arm_commander, log, setup_teardown):
        target = arm_commander.arm_mvgroup.get_random_pose()
        broadcast_pose(arm_commander.tf_broadcaster,target.pose,'target','world')
        res, _ = arm_commander.go_pose(target)
        assert res , "motion planning failed"
        assert all_close(target, arm_commander.get_end_effector_pose(), GOAL_TOLERANCE)

class TestIKPoints:
    def test_go_pose_ikpoints_dist(self, arm_commander, log, setup_teardown):
        target = arm_commander.arm_mvgroup.get_random_pose()
        broadcast_pose(arm_commander.tf_broadcaster,target.pose,'target','world')
        res, _ = arm_commander.go_pose_ikpoints(target)
        assert res , "motion planning failed"
        assert_dist(arm_commander.get_end_effector_pose(), target, DIST_TOLERANCE)

    def test_go_pose_ikpoints_allclose(self, arm_commander, log, setup_teardown):
        target = arm_commander.arm_mvgroup.get_random_pose()
        broadcast_pose(arm_commander.tf_broadcaster,target.pose,'target','world')
        res, _ = arm_commander.go_pose_ikpoints(target)
        assert res , "motion planning failed"
        assert all_close(target, arm_commander.get_end_effector_pose(), GOAL_TOLERANCE)

    def test_go_position_ikpoints_dist(self, arm_commander, log, setup_teardown):
        target = arm_commander.arm_mvgroup.get_random_pose().pose.position
        broadcast_point(arm_commander.tf_broadcaster,target,'target','world')
        res, _ = arm_commander.go_position_ikpoints(target)
        assert res , "motion planning failed"
        assert_dist(arm_commander.get_end_effector_pose().pose.position, target, DIST_TOLERANCE)

    def test_go_position_ikpoints_allclose(self, arm_commander, log, setup_teardown):
        target = arm_commander.arm_mvgroup.get_random_pose().pose.position
        broadcast_point(arm_commander.tf_broadcaster,target,'target','world')
        res, _ = arm_commander.go_position_ikpoints(target)
        assert res , "motion planning failed"
        assert all_close(target, arm_commander.get_end_effector_pose().pose.position, GOAL_TOLERANCE)

@pytest.mark.parametrize("weights", [[0.25,0.25,0.25,1,1,1],[0.1,0.1,0.1,1,1,1],[0.05,0.05,0.05,1,1,1],[0,0,0,1,1,1],[1,1,1,0,0,0]])
class TestMatlabIK:

    def test_go_position_matlabik_dist(self, arm_commander, log, setup_teardown, weights):
        target = arm_commander.arm_mvgroup.get_random_pose()
        broadcast_pose(arm_commander.tf_broadcaster,target.pose,'target','world')
        res, _, joints = arm_commander.go_position_matlabik(target.pose.position)
        assert res , "motion planning failed"
        assert all_close(joints, arm_commander.arm_mvgroup.get_current_joint_values(), JOINT_TOLERANCE) , "joint target not within tolerance"
        assert_dist(arm_commander.get_end_effector_pose().pose.position,target.pose.position, DIST_TOLERANCE)

    def test_go_pose_matlabik_allclose(self, arm_commander, log, setup_teardown, weights):
        target = arm_commander.arm_mvgroup.get_random_pose()
        broadcast_pose(arm_commander.tf_broadcaster,target.pose,'target','world')
        res, _, joints = arm_commander.go_pose_matlabik(target, weights)
        assert res , "motion planning failed"
        assert all_close(joints, arm_commander.arm_mvgroup.get_current_joint_values(), JOINT_TOLERANCE) , "joint target not within tolerance"
        assert all_close(target, arm_commander.get_end_effector_pose(), GOAL_TOLERANCE)
        # log.info(all_close(target, arm_commander.get_end_effector_pose(), GOAL_TOLERANCE))
        # log.info(all_close(target.pose.position, arm_commander.get_end_effector_pose().pose.position, GOAL_TOLERANCE))
        # log.info(all_close(target.pose.orientation, arm_commander.get_end_effector_pose().pose.orientation, GOAL_TOLERANCE))

    def test_go_pose_matlabik_dist(self, arm_commander, log, setup_teardown, weights):
        target = arm_commander.arm_mvgroup.get_random_pose()
        broadcast_pose(arm_commander.tf_broadcaster,target.pose,'target','world')
        res, _, joints = arm_commander.go_pose_matlabik(target, weights)
        assert res , "motion planning failed"
        assert_dist(arm_commander.get_end_effector_pose(),target, DIST_TOLERANCE)

class TestRelaxedIK:
    def test_go_pose_relaxedik_dist(self, arm_commander, log, setup_teardown):
        target = arm_commander.arm_mvgroup.get_random_pose()
        broadcast_pose(arm_commander.tf_broadcaster,target.pose,'target','world')
        res, _, joints = arm_commander.go_pose_relaxedik(target)
        assert res , "motion planning failed"
        assert all_close(joints, arm_commander.arm_mvgroup.get_current_joint_values(), JOINT_TOLERANCE) , "joint target not within tolerance"
        assert_dist(arm_commander.get_end_effector_pose(),target, DIST_TOLERANCE)

    def test_go_pose_relaxedik_allclose(self, arm_commander, log, setup_teardown):
        target = arm_commander.arm_mvgroup.get_random_pose()
        broadcast_pose(arm_commander.tf_broadcaster,target.pose,'target','world')
        res, _, joints = arm_commander.go_pose_relaxedik(target)
        assert res , "motion planning failed"
        assert all_close(joints, arm_commander.arm_mvgroup.get_current_joint_values(), JOINT_TOLERANCE) , "joint target not within tolerance"
        assert all_close(target, arm_commander.get_end_effector_pose(), GOAL_TOLERANCE)

class TestKinpyIK:
    def test_go_pose_kinpy_dist(self, arm_commander, log, setup_teardown):
        rp = rospkg.RosPack()
        urdf_path = rp.get_path('ob1_arm_description') + "/urdf/main.urdf"
        kinpy_arm = kp.build_serial_chain_from_urdf(
            open(urdf_path).read(),
            root_link_name="ob1_arm_fixed_base_link",
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
        assert_dist(pose_goal, arm_commander.get_end_effector_pose().pose, DIST_TOLERANCE)

################### OBJECT MANIPULATION TESTS ######################
@pytest.mark.parametrize("sphere_radius", [0.03])
class TestPickSphere:
    def test_go_scene_object(self, arm_commander, log, setup_teardown, sphere_radius):
        spawn_random_sphere(arm_commander, sphere_radius)
        objs = arm_commander.scene.get_objects()
        assert len(objs) > 0, "Could not find any scene objects"
        obj:CollisionObject = list(objs.items())[0][1]
        broadcast_pose(arm_commander.tf_broadcaster,obj.pose,'target','world')
        res, target, joint_target = arm_commander.go_position(obj.pose.position)
        assert res , "motion planning failed"
        assert all_close(joint_target, arm_commander.arm_mvgroup.get_current_joint_values(), JOINT_TOLERANCE), "joints not within tolerance"
        assert_dist(arm_commander.get_end_effector_pose().pose.position, obj.pose.position, DIST_TOLERANCE)

    def test_pick_sphere(self, arm_commander, log, setup_teardown, sphere_radius):
        spawn_random_sphere(arm_commander, sphere_radius)
        objs = arm_commander.scene.get_objects()
        assert len(objs) > 0, "Could not find any scene objects"
        obj:CollisionObject = list(objs.items())[0][1]
        broadcast_pose(arm_commander.tf_broadcaster,obj.pose,'target','world')
        assert arm_commander.pick_object(obj)

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

@pytest.mark.parametrize("pick_tolerance", [0.01, 0.02, 0.03, 0.04, 0.05])
def test_pick_cylinder_and_move(arm_commander:ArmCommander, log, setup_teardown, pick_tolerance):
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
def test_pick_and_place_cylinder(arm_commander, log, setup_teardown,attempts, pick_tolerance, place_tolerance):
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

def test_pick_and_place_cylinder_in_region(arm_commander, log, setup_teardown):
    pick_target = PoseStamped()
    pick_target.header.frame_id = arm_commander.robot.get_planning_frame()
    pick_target.pose.position = arm_commander.arm_mvgroup.get_random_pose().pose.position
    pick_target.pose.orientation.w = 1
    arm_commander.scene.add_cylinder('cylinder', pick_target, height=0.15, radius=0.03)

    time.sleep(2)

    obj = arm_commander.get_object('cylinder')
    assert obj!=None, 'could not find any scene objects'
    broadcast_pose(arm_commander.tf_broadcaster,obj.pose,'target','world')

    assert arm_commander.pick_object(obj, attempts=10, pick_tolerance=0.01), 'failed to pick object'

    dx,dy,dz = 0.2, 0.2, 0.01
    req = IKPointsServiceRequest()
    req.request = 'get up to dist targets'
    req.distance = 0.2
    req.pose = pick_target.pose
    pose_targets , joint_targets, _ = ikpoints_service_client(req)
    log.info("%d ik targets queried" % len(pose_targets))


    xc,yc,zc = tuple(point_to_list(pick_target.pose.position))
    xmin, xmax, ymin, ymax, zmin, zmax = xc-dx/2, xc+dx/2, yc-dy/2, yc+dy/2, zc-dz/2, zc+dz/2
    def in_cube(point):
        x,y,z = tuple(point)
        return x >= xmin and x <= xmax and y >= ymin and y <= ymax and z >= zmin and z <= zmax

    pose_targets = [pose for pose in pose_targets if in_cube(point_to_list(pose.position))]
    log.info("%d ik targets left in cube" % len(pose_targets))
    QuaternionComparators.set_ref_quaternion(Quaternion(0,0,0,1))

    #first we should get the tf from object --> eef : t_obj_eef (numpy matrix 4x4)
    t_obj_w = np.linalg.inv(pose_to_mat(pick_target.pose))
    t_w_eef = pose_to_mat(arm_commander.get_end_effector_pose().pose)
    #this should be constant until the detach operation
    t_obj_eef = np.dot(t_obj_w, t_w_eef)

    QuaternionComparators.set_ref_transform(t_obj_eef)
    indices = QuaternionComparators.sort_indices(pose_targets,4)

    res = False
    for i in indices:
        pt = pose_targets[i]
        broadcast_pose(arm_commander.tf_broadcaster,pt,'target','world')
        joints = joint_targets[i]
        res = arm_commander.go_joint(joints)[0]
        if res:
            break
    
    assert res
    assert arm_commander.detach_object(obj)

def test_go_cylinder_matlab(arm_commander, log, setup_teardown, attempts=100):
    p = PoseStamped()
    p.header.frame_id = arm_commander.robot.get_planning_frame()
    p.pose.position = arm_commander.arm_mvgroup.get_random_pose().pose.position
    p.pose.orientation.w = 1
    arm_commander.scene.add_cylinder('cylinder', p, height=0.15, radius=0.03)

    time.sleep(2)

    obj = arm_commander.get_object('cylinder')
    assert obj!=None, 'could not find any scene objects'
    broadcast_pose(arm_commander.tf_broadcaster,obj.pose,'target','world')

    res = False
    for _ in range(attempts):
        res = arm_commander.go_pose_matlabik(p, [0.01,0.01,0.01,1,1,1])[0]
        if res:
            break
    assert res

def test_pick_and_place_cylinder_in_region_matlab(arm_commander, log, setup_teardown):
    pick_target = PoseStamped()
    pick_target.header.frame_id = arm_commander.robot.get_planning_frame()
    pick_target.pose.position = arm_commander.arm_mvgroup.get_random_pose().pose.position
    pick_target.pose.orientation.w = 1
    arm_commander.scene.add_cylinder('cylinder', pick_target, height=0.15, radius=0.03)

    time.sleep(2)

    obj = arm_commander.get_object('cylinder')
    assert obj!=None, 'could not find any scene objects'
    broadcast_pose(arm_commander.tf_broadcaster,obj.pose,'target','world')

    assert arm_commander.pick_object(obj, attempts=10, pick_tolerance=0.01), 'failed to pick object'

    #first we should get the tf from object --> eef : t_obj_eef (numpy matrix 4x4)
    t_obj_w = np.linalg.inv(pose_to_mat(obj.pose))
    t_w_eef = pose_to_mat(arm_commander.get_end_effector_pose().pose)
    #this should be constant until the detach operation
    t_obj_eef = np.dot(t_obj_w, t_w_eef)

    place_target = PoseStamped()
    place_target.header.frame_id = arm_commander.robot.get_planning_frame()
    place_target.pose.position = arm_commander.arm_mvgroup.get_random_pose().pose.position
    place_target.pose.orientation.w = 1
    broadcast_pose(arm_commander.tf_broadcaster,place_target.pose,'target','world')

    t_w_pobj = pose_to_mat(place_target.pose)
    t_w_peef = np.dot(t_w_pobj, t_obj_eef)
    place_target.pose = mat_to_pose(t_w_peef)
    broadcast_pose(arm_commander.tf_broadcaster,place_target.pose,'eef','world')

    assert arm_commander.go_pose_matlabik(place_target, [1,1,1,0,0,0])[0]

    assert arm_commander.detach_object(obj)
    time.sleep(5)


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