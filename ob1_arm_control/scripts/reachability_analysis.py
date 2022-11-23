#! /usr/bin/env python
from __future__ import print_function
import time
import rospy
from copy import deepcopy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Pose, PoseStamped, Point, Quaternion
from std_msgs.msg import ColorRGBA
import pickle
from arm_commander import ArmCommander
import rospkg
from dataclasses import dataclass
import kinpy as kp
import os
import numpy as np
import sys
import h5py

# Author: Yousif El-Wishahy
# Email: yel.wishahy@gmail.com
# Reachability analysis module to test/analyze UBC Open Robotics arm IK algorithms

############ PARAMETERS ##################
#auto save interval (hours)
AUTO_SAVE_INTERVAL = 0.1

#reachability test joint space resolution, in radians
#note: there are 5 joints, so small resolutions will produce a LARGE amount of test values...
JS_RESOLUTION = 0.23

ARM_JOINT_LIMITS = [(-1.5787, 1.5787), (-1.5787, 1.5787), (-3.14, 3.14), (-1.5787, 1.5787), (-3.14, 3.14)]

rp = rospkg.RosPack()
PACKAGE_PATH = rp.get_path('ob1_arm_control')
###########################################

#global list to store all joint targets to test
untested_joint_targets = []

#global variables to store current successful joint target data
data_flag = False

DATA_KEYS = ['pose_data', 'joint_data', 'point_data']
pose_data = None
joint_data = None
point_data = None

def save_data_h5(h5f:h5py.File):
    def append_data(key, data):
        h5f[key].resize((h5f[key].shape[0] + data.shape[0]), axis = 0)
        h5f[key][-data.shape[0]:] = data
    if not data_flag:
        return
    for key in DATA_KEYS:
        data = np.array([globals()[key],]) #need to make an array of the data
        if key in h5f:
            append_data(key, data)
        else:
            n_data = data.shape[1]
            h5f.create_dataset(key, data=data, chunks=True, maxshape=(None,n_data))
        globals()[key] = None

def calculate_js_reachability(fk_planner="MOVEIT"):
    """
    Function for calcualting reachability in joint space.
    Iterates over joint goals with a certain resolution and saves Pose results as green or red markers

    @param fk_planner = 'MOVEIT' or 'KINPY'
    """

    if fk_planner == "MOVEIT":
        #arm commander class, intializes move group planning interface
        arm:ArmCommander = ArmCommander(sample_attempts=1, sample_timeout=0.1,goal_tolerance=0.01)
        # ArmCommander(sample_attempts=5, sample_time_out=5,goal_tolerance=0.01)
    elif fk_planner == "KINPY":
        #arm commander class, kinpy kinematics serial chain
        rp = rospkg.RosPack()
        urdf_path = rp.get_path('ob1_arm_description') + "/urdf/main.urdf"
        arm = kp.build_serial_chain_from_urdf(
            open(urdf_path).read(),
            root_link_name="ob1_arm_base_link",
            end_link_name="ob1_arm_eef_link"
        )

    num_joints = len(ARM_JOINT_LIMITS)
    joint_target = num_joints * [0]
    def generate_joint_space_recursively(joint_index=1):
        """
        recursive helper method to generate the test space of joint values
        """

        if joint_index <= num_joints:
            min,max = ARM_JOINT_LIMITS[joint_index-1]
            val = min
            while val < max:
                val+=JS_RESOLUTION
                if val >= min and val <= max:
                    joint_target[joint_index-1] = val 
                    untested_joint_targets.append(deepcopy(joint_target))
                generate_joint_space_recursively(joint_index+1)
        return

    search_start = time.time()
    print("============ Generating Joint Space Recursively...")
    generate_joint_space_recursively()
    search_duration = time.time() - search_start
    print("============ Generated %s joint targets in %s seconds" %(len(untested_joint_targets), search_duration))

    with h5py.File(PACKAGE_PATH+"/data/ik_data.h5", "w") as h5f:
        counter = 0
        success_counter = 0
        total = len(untested_joint_targets)
        time_remaining = 0
        time_passed = 0
        end_flag = False
        def check_end_loop():
            print("test targets left: %d"%len(untested_joint_targets))
            if len(untested_joint_targets) == 0:
                return True
            return False
        while not end_flag:
            #update loop conditions
            if rospy.is_shutdown():
                break
            current_joints = untested_joint_targets.pop()
            end_flag = check_end_loop()

            start_t = time.time()
            print("Testing joint target %s/%s " %(counter,total))
            success = False
            fk_sol = None
            p = None
            pq = None
            global point_data
            global joint_data
            global pose_data
            global data_flag

            if fk_planner == 'MOVEIT':
                if arm.go_joint(current_joints)[0]:
                    success = True
                    success_counter+=1
                    eef_pose = arm.get_end_effector_pose().pose
                    p = eef_pose.position
                    p = np.array([p.x, p.y, p.z])
                    q = eef_pose.orientation
                    q = np.array([q.x, q.y, q.z, q.w])
                    pq = np.append(p,q)
            elif fk_planner == 'KINPY':
                fk_sol = arm.forward_kinematics(current_joints)
                if fk_sol is not None and type(fk_sol) is kp.Transform:
                    success_counter+=1
                    success = True
                    p = fk_sol.pos
                    pq = np.append(p,fk_sol.rot)
            if success:
                data_flag = True
                pose_data = pq
                point_data = p
                joint_data = np.array(current_joints)
                save_data_h5(h5f)
            data_flag = False
   
            planning_t = time.time() - start_t
            counter+=1
            time_passed += (planning_t/3600)
            time_remaining = time_passed/counter * (total-counter)

            print(" %s/%s Planning Success Rate " %(success_counter,counter))
            print("Time remaining: Est. %s hours" %(time_remaining))
            

if __name__ == '__main__':
    calculate_js_reachability(fk_planner='KINPY')