"""
Author: Yousif El-Wishahy
Email: yel.wishahy@gmail.com 

Transform and transform tree related helper functions for Ob1 Arm control and test software
"""

import tf
from tf import transformations
import rospy
from geometry_msgs.msg import *
import numpy as np
import geometry_msgs.msg as gm

def convert_to_list(obj):
    """
    @brief helper function to convert geometry messages to list [x,y,z]

    returns none if invalid input object
    """
    if type(obj) is gm.Point:
        return point_to_list(obj)
    if type(obj) is gm.Quaternion:
        return quat_to_list(obj)
    elif type(obj) is gm.Pose:
        return pose_to_list(obj)
    elif type(obj) is gm.PoseStamped:
        return pose_to_list(obj.pose)
    elif type(obj) is np.ndarray:
        return obj.tolist()
    elif type(obj) is list:
        return obj
    else:
        raise ValueError("Invalid input to convert_to_list funtion (%s)" % type(obj))

def quat_to_list(q:Quaternion):
    """
    @return geometry_msgs/Quaterion q as [x,y,z,w]
    """
    assert type(q) is Quaternion
    return [q.x,q.y,q.z,q.w]

def point_to_list(p:Point):
    """
    @return geometry_msgs/Point p as [x,y,z]
    """
    assert type(p) is Point
    return [p.x,p.y,p.z]

def pose_to_list(pose:Pose):
    """
    @return geometry_msgs/Pose p as [px,py,pz,qx,qy,qz,qw]
    """
    assert type(pose) is Pose
    return point_to_list(pose.position) + quat_to_list(pose.orientation)

def broadcast_pose(tf_broadcaster:tf.TransformBroadcaster, pose:Pose,child:str,parent:str):
    """
    @brief helper function to broadcase pose to transform tree
    @param pose: rospy Pose() object
    @param child: child tf frame name
    @param parent: parent tf frame name
    """
    obj_tr = pose.position
    obj_tr = (obj_tr.x,obj_tr.y,obj_tr.z)
    obj_rot = pose.orientation
    obj_rot = (obj_rot.x,obj_rot.y,obj_rot.z,obj_rot.w)
    tf_broadcaster.sendTransform(obj_tr, obj_rot, rospy.Time.now(), child, parent)

def broadcast_point(tf_broadcaster:tf.TransformBroadcaster, point, child:str,parent:str):
    """
    @brief helper function to broadcase point to transofrm tree
    @param pose: Point object , or list [x,y,z], or tuple (x,y,z) or np.array([x,y,z])
    @param child: child tf frame name
    @param parent: parent tf frame name
    """
    if type(point) is tuple:
        obj_tr = point
    elif type(point) is list:
        obj_tr = tuple(point)
    elif type(point) is Point:
        obj_tr = (point.x,point.y,point.z)
    elif type(point) is np.ndarray:
        obj_tr = tuple(point.tolist())
    obj_rot = (0,0,0,1)
    tf_broadcaster.sendTransform(obj_tr, obj_rot, rospy.Time.now(), child, parent)

def point_to_mat(p:Point):
    """@brief convert a ros point to 4x4 ndarray matrix"""
    return transformations.translation_matrix((p.x,p.y,p.z))


def quat_to_mat(q:Quaternion):
    """@brief convert a ros quaternion to 4x4 ndarray matrix"""
    return transformations.quaternion_matrix((q.x,q.y,q.z,q.w))

def pose_to_mat(p:Pose):
    """@brief convert a ros pose to 4x4 ndarray matrix"""
    return np.dot(point_to_mat(p.position), quat_to_mat(p.orientation))

def mat_to_pose(mat44):
    """@brief convert a 4x4 ndarray matrix to ros pose"""
    pos = tuple(transformations.translation_from_matrix(mat44))[:3]
    quat = tuple(transformations.quaternion_from_matrix(mat44))
    return Pose(Point(*pos), Quaternion(*quat))

def t_r_to_mat(translation, rotation):
    """@brief convert translation list and quaternion list to tf matrix"""
    return np.dot(transformations.translation_matrix(translation), transformations.quaternion_matrix(rotation))

def get_tf_p1_p2(p1:Pose, p2:Pose):
    """@brief get tranformation matrix from p1 to p2"""
    p1_mat44 = pose_to_mat(p1)
    p1_invmat44 = np.linalg.inv(p1_mat44)
    p2_mat44 = pose_to_mat(p2)
    return np.dot(p1_invmat44, p2_mat44)

def transform_pose(tf, p:Pose):
    """@brief transform pose p with transform matrix tf"""
    p_mat44 = pose_to_mat(p)
    tp_mat44 = np.dot(tf, p_mat44)
    return mat_to_pose(tp_mat44)

def transform_pose_with_tr(translation, rotation, pose:Pose):
    """
    @brief helper function to transform a pose with a 4x4 transformation matrix
    @param translation: [x,y,z]
    @param rotation: quaternion [x,y,z,w]
    @param pose: geometry_msgs/Pose object
    @return transformed pose

    **Logic**
    Pass in translation and rotation from frame1 to frame2
    Create 4x4 tf matrix from this
    Create 4x4 matrix of pose
    tf dot pose
    extract trans and quat from result
    """
    tf_mat44 = t_r_to_mat(translation, rotation)
    pose_mat44 = pose_to_mat(pose)
    #apply transformation
    tpose_44 = np.dot(tf_mat44,pose_mat44)
    return mat_to_pose(tpose_44)