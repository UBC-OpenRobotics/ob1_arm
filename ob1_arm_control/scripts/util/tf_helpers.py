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
import pyquaternion as pyq
from types import FunctionType
from functools import cmp_to_key

class QuaternionComparators:
    """@brief a class of comparators for comparing quaternions to a ref quaternion"""
    _ref_quaternion:Quaternion = None
    _ref_transform:np.ndarray = None
    _comparators:list = []

    @staticmethod
    def set_ref_quaternion(q:Quaternion):
        QuaternionComparators._ref_quaternion = q

    @staticmethod
    def set_ref_transform(tf):
        QuaternionComparators._ref_transform = tf

    @staticmethod
    def comparators():
        """returns a list of the quaternion comparator functions contained in this class"""
        if len(QuaternionComparators._comparators) < 1:
            qc = QuaternionComparators
            QuaternionComparators._comparators = [qc.q_comp_pyq_0, qc.q_comp_ang_1, qc.q_comp_xyz_2, qc.q_comp_w_3, qc.q_comp_transform_pyq_4]
        return QuaternionComparators._comparators
    
    @staticmethod
    def get_comparator(id:int):
        for cmp in QuaternionComparators.comparators():
            if str(id) in cmp.__name__:
                return cmp
        raise ValueError("Could not find comparator with id: %d. The list of comparators is %s." % (id, QuaternionComparators.comparators()))
    
    @staticmethod
    def sort_indices(tosort, comparator):
        assert type(tosort) is list
        if type(comparator) is int:
            cmp = QuaternionComparators.get_comparator(comparator)
        elif type(comparator) is FunctionType:
            cmp = comparator
        else:
            raise ValueError("comparator is not the right type: %s" % type(comparator))
        
        assert type(tosort) is list
        assert len(tosort) > 0
        if type(tosort[0]) is Quaternion:
            tosort = [(i, tosort[i]) for i in range(len(tosort))]
        elif type(tosort[0]) is Pose:
            tosort = [(i, tosort[i].orientation) for i in range(len(tosort))]
        elif type(tosort[0]) is not tuple:
            raise ValueError("list is not the right type: %s" % type(tosort[0]))

        tosort = sorted(tosort, key=cmp_to_key(lambda item1, item2: cmp(item1[1], item2[1])))
        return [item[0] for item in tosort]

    @staticmethod
    def sort(tosort, comparator):
        indices = QuaternionComparators.sort_indices(list(tosort), comparator)
        return [tosort[i] for i in indices]
 
    @staticmethod
    def q_comp_pyq_0(q1:Quaternion, q2:Quaternion):
        """
        comparator using absolute distance between quaternions
        def: http://kieranwynn.github.io/pyquaternion/#distance-computation
        """
        q = QuaternionComparators._ref_quaternion
        assert type(q) is Quaternion
        Q = pyq.Quaternion(q.w, q.x, q.y, q.z)
        q1 = quat_to_list(q1)
        q2 = quat_to_list(q2)
        Q1 = pyq.Quaternion(q1[3], q1[0], q1[1], q1[2])
        Q2 = pyq.Quaternion(q2[3], q2[0], q2[1], q2[2])
        d1 = pyq.Quaternion.absolute_distance(Q,Q1)
        d2 = pyq.Quaternion.absolute_distance(Q,Q2)
        return d1-d2

    @staticmethod
    def q_comp_ang_1(q1:Quaternion, q2:Quaternion):
        """comparator using quaternion's cos_phi_half / inner product value"""
        q = QuaternionComparators._ref_quaternion
        assert type(q) is Quaternion
        q = quat_to_list(q)
        def cos_phi_half(q1,q2):
            f = q1[0]*q2[0] + q1[1]*q2[1] + q1[2]*q2[2] + q1[3]*q2[3]
            return np.fabs(f)
        q1 = quat_to_list(q1)
        q2 = quat_to_list(q2)
        return cos_phi_half(q,q1) - cos_phi_half(q,q2)

    @staticmethod
    def q_comp_xyz_2(q1:Quaternion, q2:Quaternion):
        """comparator using roational axis of quaternions"""
        q = QuaternionComparators._ref_quaternion
        assert type(q) is Quaternion
        q_axis = np.array(quat_to_list(q)[:3])
        q1_axis = np.array(quat_to_list(q1)[:3])
        q2_axis = np.array(quat_to_list(q2)[:3])
        d1 = np.linalg.norm(q_axis-q1_axis)
        d2 = np.linalg.norm(q_axis-q2_axis)
        return d1-d2

    @staticmethod
    def q_comp_w_3(q1:Quaternion, q2:Quaternion):
        """comparator using roational axis of quaternions"""
        q = QuaternionComparators._ref_quaternion
        assert type(q) is Quaternion
        d1 = np.abs(q.w - q1.w)
        d2 = np.abs(q.w - q2.w)
        return d1-d2
    
    @staticmethod
    def q_comp_transform_pyq_4(q1, q2):
        tf = QuaternionComparators._ref_transform
        assert tf is not None
        assert type(q1) is Quaternion
        assert type(q2) is Quaternion

        q1 = mat_to_pose(np.dot(quat_to_mat(q1), tf)).orientation
        q2 = mat_to_pose(np.dot(quat_to_mat(q2), tf)).orientation

        return QuaternionComparators.q_comp_pyq_0(q1, q2)

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