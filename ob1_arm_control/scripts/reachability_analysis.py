#!/usr/bin/env python3.8
from __future__ import print_function
import time
import rospy
from copy import deepcopy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Pose, PoseStamped
from std_msgs.msg import ColorRGBA
import pickle
from arm_commander import ArmCommander
import rospkg
from dataclasses import dataclass

# Author: Yousif El-Wishahy
# Email: yel.wishahy@gmail.com
# Reachability analysis module to test/analyze UBC Open Robotics arm IK algorithms

############ PARAMETERS ##################
#auto save interval (hours)
AUTO_SAVE_INTERVAL = 0.1

#reachability test joint space resolution, in radians
#note: there are 4+ joints, so small resolutions will produce a LARGE amount of test values...
JS_RESOLUTION = 0.4 #0.5

###########################################

#arm commander class, intializes move group planning interface
arm_commander:ArmCommander = ArmCommander(sample_attempts=1, sample_time_out=0.1,goal_tolerance=0.01)
# ArmCommander(sample_attempts=5, sample_time_out=5,goal_tolerance=0.01)

rp = rospkg.RosPack()
PACKAGE_PATH = rp.get_path('ob1_arm_control')

#std_msgs.mg.ColorRGBA ros clr msg for visual markers
GREEN = ColorRGBA(r=0, g=255, b=0, a=1)
RED = ColorRGBA(r=255, g=0, b=0, a=1)

# Ros Marker message (visualization_msgs.msg.Marker) to store points in cartesian space
# IK reachability is indicated by point colour
# red = non reachable with IK algorithm
# green = reachable with IK algorithm 
marker_data:Marker = Marker()

#pose data
#stores all successful orientations and positions (aka poses)
successful_pose_data = []

#stores all failed orientations and positions (aka poses)
failed_pose_data = []

#list to store all joint targets to test
joint_targets = []

#list to store failed joint targets
failed_joint_targets = []

#list to store successful joint targets
successful_joint_targets = []

#list to store combined data of successful pose and joint target relations
ikpoints_list = []

def save_marker_data():
    """
    Saves the test data into pickle files

    marker_data.pickle is a saved visualization_msgs.msg.Marker object
    pose_data.pickle is a saved list of PoseStamped objects
    joint_data.pickle is a dictionary containing lists of good and bad joint targets
    """

    with open(PACKAGE_PATH+"/data/marker_data.pickle", "wb") as output_file:
        pickle.dump(marker_data, output_file)
    with open(PACKAGE_PATH+"/data/pose_data.pickle", "wb") as output_file:
        pickle.dump({"good poses":successful_pose_data,
        "bad poses":failed_pose_data}, output_file)
    with open(PACKAGE_PATH+"/data/joint_data.pickle", "wb") as output_file:
        pickle.dump({"good joint targets":successful_joint_targets,
        "bad joint targets":failed_joint_targets}, output_file)
    with open(PACKAGE_PATH+"/data/ikpoints_data.pickle", "wb") as output_file:
        pickle.dump(ikpoints_list, output_file)

def calculate_js_reachability():
    """
    Function for calcualting reachability in joint space.
    Iterates over joint goals with a certain resolution and saves Pose results as green or red markers
    """
    joint_target = arm_commander.arm_mvgroup.get_current_joint_values()
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
    last_autosave = 0
    for joints in joint_targets:
        print("Testing joint target %s/%s " %(counter,total))
        print(joints)
        start_t = time.time()
        success, _ = arm_commander.go_joint(joints)
        planning_t = time.time() - start_t
        counter+=1
        time_passed += (planning_t/3600)
        time_remaining = time_passed/counter * (total-counter)
        if success:
            success_counter+=1
            eef_pose_stamped = arm_commander.get_end_effector_pose()
            position = eef_pose_stamped.pose.position
            header = eef_pose_stamped.header

            successful_pose_data.append(eef_pose_stamped)
            marker_data.points.append(position)
            marker_data.colors.append(GREEN)
            marker_data.header = header

            successful_joint_targets.append(joints)

            ikpoint = {"pose_stamped":eef_pose_stamped,"joint_target":joints}
            ikpoints_list.append(ikpoint)
        else:
            failed_joint_targets.append(joints)

        if time_passed > last_autosave + AUTO_SAVE_INTERVAL:
            save_marker_data()
            last_autosave = time_passed
        
        if rospy.is_shutdown():
            return

        print(" %s/%s Planning Success Rate " %(success_counter,counter))
        print("Time remaining: Est. %s hours" %(time_remaining))
        print("Time to Autosave: %s hours\n" %(last_autosave + AUTO_SAVE_INTERVAL - time_passed))
    
    save_marker_data()

# def combine_pickles():
#     with open(PACKAGE_PATH+"/data/15k_ikpoints_data.pickle","rb") as input_file:
#         data_15k = pickle.load(input_file)
#     with open(PACKAGE_PATH+"/data/5k_ikpoints_data.pickle","rb") as input_file:
#         data_5k = pickle.load(input_file)
#     with open(PACKAGE_PATH+"/data/20k_ikpoints_data.pickle", "wb") as output_file:
#         pickle.dump(data_15k + data_5k, output_file)

if __name__ == '__main__':
    calculate_js_reachability()