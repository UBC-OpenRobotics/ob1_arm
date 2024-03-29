from moveit_commander.conversions import pose_to_list
import numpy as np
from math import dist, fabs, cos
import geometry_msgs
from geometry_msgs.msg import PoseStamped
from util.tf_helpers import *
import time

def spawn_random_sphere(arm_commander, r):
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

def clear_scene(arm_commander):
    objs = arm_commander.scene.get_objects()
    for o in list(objs.items()):
        obj= o[1]
        arm_commander.detach_object(obj)
    arm_commander.scene.clear()

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
    if type(goal) is np.ndarray:
        return all_close(goal.tolist(), actual.tolist(), tolerance)

    if type(goal) is list:
        for index in range(len(goal)):
            if abs(actual[index] - goal[index]) > tolerance:
                return False

    elif type(goal) is geometry_msgs.msg.Quaternion:
        qx0, qy0, qz0, qw0 = quat_to_list(actual)
        qx1, qy1, qz1, qw1 = quat_to_list(goal)
        cos_phi_half = fabs(qx0 * qx1 + qy0 * qy1 + qz0 * qz1 + qw0 * qw1)
        return cos_phi_half >= cos(tolerance / 2.0)
    
    elif type(goal) is geometry_msgs.msg.Point:
        x0, y0, z0 = point_to_list(actual)
        x1, y1, z1 = point_to_list(goal)
        d = dist((x1, y1, z1), (x0, y0, z0))
        return d <= tolerance

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

def avg_all_close(goal, actual):
    if type(goal) != list:
        goal = convert_to_list(goal)
        actual = convert_to_list(actual)
    total = 0
    for index in range(len(goal)):
        total += abs(actual[index] - goal[index])
    return total/len(goal)

def euc_distance(p1,p2):
    assert type(p1) is not Quaternion
    p1 = np.array(convert_to_list(p1)[:3])
    p2 = np.array(convert_to_list(p2)[:3])
    return np.linalg.norm(p1-p2)

def assert_dist(current_state,target,tolerance:float):
    assert type(current_state) == type(target), "current state and target state types do not match!"
    if type(target) is PoseStamped and type(current_state) is PoseStamped:
        assert target.header.frame_id == current_state.header.frame_id , "transform frame mismatch"
    d = euc_distance(current_state,target)
    assert d <= tolerance, "Distance from target is too far, %f cm" % (d*100)

def check_dist(current_state,target,tolerance:float):
    if type(target) != type(current_state):
        return False, "target type is not the same as current state type"
    if type(target) is PoseStamped and type(current_state) is PoseStamped:
        if target.header.frame_id != current_state.header.frame_id:
            return False, "transform frame mismatch. Object pose is in %s frame" % target.header.frame_id
    d = euc_distance(current_state,target)
    if d <= tolerance: 
        return True, d, "Motion planning tolerance success. %f cm" % (d*100)
    else:
        return False, d, "Distance from target is too far, %f cm" % (d*100)