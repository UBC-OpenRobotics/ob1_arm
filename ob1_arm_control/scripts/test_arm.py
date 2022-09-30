#!/usr/bin/env python3.8
from __future__ import print_function
import pickle
import time
from arm_commander import ArmCommander, assert_ik_result
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

##TEST PARAMS
rp = rospkg.RosPack()
PACKAGE_PATH = rp.get_path('ob1_arm_control')
DATA_FILE_PATH = PACKAGE_PATH+"/data/5k_pose_data.pickle"

HOR = '-'*20

TEST_TOLERANCE = 0.05

def go_pose_test(arm_commander: ArmCommander,iterations):
    """go to random poses generated within reach and test end effector pose accuracy after ik motion complete"""
    success_counter = 0
    counter = 0
    for _ in range(iterations):
        res, target = arm_commander.go_pose()
        # print(arm_commander.pose_goal)
        # print(arm_commander.get_end_effector_pose())
        if arm_commander.compare_eef_pose_states():
            success_counter+=1
        counter+=1

        print("Iteration: %s \n%s/%s goal successes" %(counter, success_counter, iterations))

def go_position_test(arm_commander: ArmCommander,iterations):
    counter = 0
    for _ in range(iterations):
        pos = arm_commander.go_position()
        print(pos)
        print(arm_commander.get_end_effector_pose())
        counter+=1

        print("Iteration: %s / %s \n" %(counter, iterations))

def go_ik_test_rand(arm_commander: ArmCommander,iterations):
    """go to random poses generated anywhere and test end effector pose accuracy after ik motion complete"""
    poses = []
    pose = Pose()
    angles = np.linspace(-2*np.pi,2*np.pi,iterations)
    positions = np.linspace(-2,2,iterations)

    #generate all poses in range
    for x in positions:
        for y in positions:
            for z in positions:
                for r in angles:
                    for p in angles:
                        for y in angles:
                            quat = quaternion_from_euler(r, p, y)
                            pose.position.x = x
                            pose.position.y = y
                            pose.position.z = z
                            pose.orientation.x = quat[0]
                            pose.orientation.y = quat[1]
                            pose.orientation.z = quat[2]
                            pose.orientation.w = quat[3]
                            poses.append(deepcopy(pose))

    for p in poses:
        print(p)
        arm_commander.go_pose(p)
        arm_commander.compare_eef_pose_states()
        if rospy.is_shutdown():
            break

def go_joint_test(arm_commander: ArmCommander,iterations=10):
    """go to random joints"""
    # for _ in range(iterations):
    #     ArmCommander.go_joint(arm_commander.arm_mvgroup)
    #     time.sleep(10)

def increment_curr_pose_go_ik_test(arm_commander: ArmCommander, d=[0,0,0.1], iterations=10):

    def get_incremented_pose_goal():
        curr_pose_stamped = arm_commander.get_end_effector_pose()

        next_pose_goal = PoseStamped()
        next_pose_goal.header.frame_id = curr_pose_stamped.header.frame_id

        next_pose_goal.pose.position = curr_pose_stamped.pose.position
        next_pose_goal.pose.position.x += d[0]
        next_pose_goal.pose.position.y += d[1]
        next_pose_goal.pose.position.z += d[2]

        next_pose_goal.pose.orientation = curr_pose_stamped.pose.orientation

        return next_pose_goal
    
    success_counter = 0
    counter = 0
    for _ in range(iterations):
        pose_goal = get_incremented_pose_goal()
        arm_commander.go_pose(pose_goal)
        if arm_commander.compare_eef_pose_states():
            success_counter+=1
        counter+=1
        print(HOR,)
        print(arm_commander._current_pose_goal)
        print(arm_commander.get_end_effector_pose())
        print("Iteration: %s \n%s/%s goal successes" %(counter, success_counter, iterations))
        print(HOR,)

def brute_force_test_translation(arm_commander: ArmCommander, range=5, d=0.01):
    pose_stamped = arm_commander.get_end_effector_pose()
    translation_range = np.arange(-1*range,range,d)

    iterations = (translation_range.size)**3
    success_counter = 0
    counter = 0
    print(HOR,)
    print("BRUTE FORCE TRANSLATION TEST\n%s ITERATIONS" % iterations)
    print(HOR,)
    #generate and test all poses in range
    for x in translation_range:
        for y in translation_range:
            for z in translation_range:
                pose_stamped.pose.position.x = x
                pose_stamped.pose.position.y = y
                pose_stamped.pose.position.z = z
                pose_stamped.pose.orientation.x = 0
                pose_stamped.pose.orientation.y = 0
                pose_stamped.pose.orientation.z = 0
                pose_stamped.pose.orientation.w = 1

                arm_commander.go_pose(pose_stamped)

                if arm_commander.compare_eef_pose_states():
                    success_counter+=1
                counter+=1
                print(HOR,)
                print(arm_commander._current_pose_goal)
                print(arm_commander.get_end_effector_pose())
                print("Iteration: %s \n%s/%s goal successes" %(counter, success_counter, iterations))
                print(HOR,)

def pose_list_test():
    #get list of poses to test
    pose_stamped_list = None    
    with open(DATA_FILE_PATH, "rb") as input_file:
        pose_stamped_list = pickle.load(input_file)
    
    if pose_stamped_list is not None and type(pose_stamped_list) is list:
        arm_commander = ArmCommander(sample_time_out=10, goal_tolerance=0.001)
        success_counter = 0
        counter = 0
        iterations = len(pose_stamped_list)

        for pose_stamped in pose_stamped_list:
            if not rospy.is_shutdown():
                print(pose_stamped)
                arm_commander.go_pose(pose_stamped)
                if arm_commander.compare_eef_pose_states():
                    success_counter+=1
                counter+=1
                print("Iteration: %s \n%s/%s goal successes" %(counter, success_counter, iterations))

def pose_list_test_position():
    #get list of poses to test
    pose_stamped_list = None    
    with open(DATA_FILE_PATH, "rb") as input_file:
        pose_stamped_list = pickle.load(input_file)
    
    if pose_stamped_list is not None and type(pose_stamped_list) is list:
        arm_commander = ArmCommander(sample_time_out=5, goal_tolerance=0.001)
        counter = 0
        iterations = len(pose_stamped_list)

        for pose_stamped in pose_stamped_list:
            if not rospy.is_shutdown():
                pos = [pose_stamped.pose.position.x, pose_stamped.pose.position.y, pose_stamped.pose.position.z]
                print(pos)
                arm_commander.go_position(pos)
                counter+=1
                print("Iteration: %s / %s goal successes\n" %(counter, iterations))

def go_ikpoints_test(arm_commander: ArmCommander,iterations):
    counter = 0
    for _ in range(iterations):
        res, pt = arm_commander.go_position_ikpoints()
        print(pt)
        print(arm_commander.get_end_effector_pose())
        counter+=1
        time.sleep(5)

        print("Iteration: %s / %s \n" %(counter, iterations))

def go_object_test():
    arm_commander = ArmCommander(sample_time_out=5, sample_attempts=5, goal_tolerance=0.001)
    objs = arm_commander.scene.get_objects()
    if len(objs) > 0:
        obj:CollisionObject = list(objs.items())[0][1]
        pos = [obj.pose.position.x,obj.pose.position.y,obj.pose.position.z]
        arm_commander.go_position_ikpoints(pos)

####################
#Main loop for testing
####################
if __name__ == '__main__':
    go_object_test()

    # pose_list_test()
    # pose_list_test_position()

    # arm_commander = ArmCommander(sample_time_out=0.1, sample_attempts=5, goal_tolerance=0.001)
    # go_ikpoints_test(arm_commander,100)
    # arm_commander.arm_mvgroup.clear_pose_targets()
    # pose = arm_commander.arm_mvgroup.get_random_pose().pose
    # pos = [pose.position.x,pose.position.y,pose.position.z]
    # arm_commander.arm_mvgroup.set_position_target(pos,"main_assembly")
    # arm_commander.arm_mvgroup.go()
    
    # go_ik_test(arm_commander,10)
    # pose_list_test()
    # pose_list_test_position()
    # go_position_test(arm_commander,100)
    # brute_force_test_translation(arm_commander,range=1,d=0.05)

    # increment_pose_go_ik_test(arm_commander)
    # arm_commander.go_joint(arm_commander.arm_mvgroup,arm_pos_joints)
    # arm_commander.go_joint(arm_commander.gripper_mvgroup,gripper_open_joints)
    # grasp_test(arm_commander,10)
    # go_ik_test(arm_commander,5)
    # goal = arm_commander.get_end_effector_pose()
    # goal.pose.position.z -= 0.01
    # arm_commander.go_ik(goal)
    # arm_commander.arm_mvgroup.shift_pose_target(2,0.05)
    # go_joint_test(arm_commander,10)


# gripper_open_joints = degrees_to_radians([-20.0, 0.0, -80.0, 0.0, -80.0])
# gripper_close_joints = degrees_to_radians([-15.0, 0.0, -2.0, 0.0, -2.0])
# arm_pos_joints = degrees_to_radians([-4.0, -29.0, -68.0, -180.0, 83.0])
