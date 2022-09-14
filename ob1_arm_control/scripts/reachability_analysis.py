#!/usr/bin/env python3.8
from __future__ import print_function
from pickletools import markobject
import sys
import time
import rospy
import moveit_commander
from moveit_commander.conversions import pose_to_list
import numpy as np
from math import pi, tau, dist, fabs, cos
import geometry_msgs
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped, Pose, Point
from moveit_commander import MoveGroupCommander,RobotCommander, PlanningSceneInterface
from moveit_msgs.msg import Grasp, GripperTranslation, MoveItErrorCodes,DisplayTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from tf.transformations import quaternion_from_euler
from copy import deepcopy, copy
from visualization_msgs.msg import Marker
from std_msgs.msg import ColorRGBA
import pickle
from arm_commander import ArmCommander
import rospkg
# Author: Yousif El-Wishahy
# Email: yel.wishahy@gmail.com
# Reachability analysis module to test/analyze UBC Open Robotics arm IK algorithms

rp = rospkg.RosPack()
PACKAGE_PATH = rp.get_path('ob1_arm_control')

############ PARAMETERS ##################
#reachability test joint space resolution, in radians
#note: there are 4+ joints, so small resolutions will produce a LARGE amount of test values...
JS_RESOLUTION = 0.95

#reachability test carteisan space resolution, in metres
CS_RESOLUTION = 0.1

#std_msgs.mg.ColorRGBA ros clr msg for visual markers
GREEN = ColorRGBA(r=0, g=165, b=8, a=0.8)
#############################################

# Ros Marker message (visualization_msgs.msg.Marker) to store points in cartesian space
# IK reachability is indicated by point colour
# red = non reachable with IK algorithm
# green = reachable with IK algorithm 
marker_data:Marker = Marker()

#pose data
#stores all successful orientations and positions (aka poses)
pose_data = []

#list to store all joint targets to test
joint_targets = []

#arm commander class, intializes move group planning interface
arm_commander:ArmCommander = ArmCommander(sample_time_out=5,goal_tolerance=0.01)

def save_marker_data():
    """
    Saves the marker msg datatype through Python's pickle method
    """
    with open(PACKAGE_PATH+"/data/marker_data.pickle", "wb") as output_file:
        pickle.dump(marker_data, output_file)
    with open(PACKAGE_PATH+"/data/pose_data.pickle", "wb") as output_file:
        pickle.dump(marker_data, output_file)

def calculate_js_reachability():
    """
    Function for calcualting reachability in joint space.
    Iterates over joint goals with a certain resolution and saves Pose results as green or red markers
    """

    joint_target = arm_commander.arm_mvgroup.get_random_joint_values()
    num_joints = len(joint_target)
    joint_targets.append(deepcopy(joint_target))

    def generate_joint_space_recursively(joint_index=1):
        """
        recursive helper method to generate the test space of joint values
        """

        if joint_index < num_joints:
            min,max = arm_commander._joint_tolerances[joint_index-1]
            val = min
            while val < max:
                val+=JS_RESOLUTION
                if val >= min and val <= max:
                    joint_target[joint_index-1] = val
                    joint_targets.append(deepcopy(joint_target))
                generate_joint_space_recursively(joint_index+1)
                if rospy.is_shutdown():
                    return
        return

    search_start = time.time()
    print("============ Generating Joint Space Recursively...")
    generate_joint_space_recursively()
    search_duration = time.time() - search_start
    print("============ Generated %s joint targets in %s seconds" %(len(joint_targets), search_duration))

    counter = 0
    success_counter = 0
    total = len(joint_targets)
    time_remaining = 0
    time_passed = 0
    for joints in joint_targets:
        print("Testing joint target %s/%s " %(counter,total))
        print(joints)
        start_t = time.time()
        success = arm_commander.go_joint(joints)
        planning_t = time.time() - start_t
        counter+=1
        time_passed += (planning_t/3600)
        time_remaining = time_passed/counter * (total-counter)
        if success:
            success_counter+=1
            eef_pose = arm_commander.get_end_effector_pose().pose
            marker_data.points.append(eef_pose.position)
            marker_data.colors.append(GREEN)
            pose_data.append(eef_pose)
            if rospy.is_shutdown():
                return
        print(" %s/%s Planning Success Rate " %(success_counter,counter))
        print("Time remaining: Est. %s hours\n" %(time_remaining))

if __name__ == '__main__':
    save_marker_data()
    try:
        calculate_js_reachability()
    except KeyboardInterrupt:
        save_marker_data() 
        exit()