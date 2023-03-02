"""
Author: Yousif El-Wishahy
Email: yel.wishahy@gmail.com 

Module containing inverse kinematics service clients to be used with MoveIT
"""

from relaxed_ik.msg import EEPoseGoals, JointAngles
from relaxed_ik.srv import RelaxedIKService, RelaxedIKServiceRequest
from ob1_arm_control.srv import MatlabIKService, MatlabIKServiceRequest
from geometry_msgs.msg import Pose
from tf import transformations
from util.tf_helpers import point_to_list, quat_to_list
import rospy

def matlabik_service_client(pose, weights, num_joints=5, timeout=10):
    """
    @brief service client function for matlab ik service

    @param pose: desired end effector geometry_msgs/Pose object for ik calculations

    @param weight: desired weights for position and orientation for ik calculation ie. [wroll, wpitch, wyaw, wx, wy, wz]
    ** position weights should be higher than orientation weights 

    @return result string, pose error, joint target list [angle1, angle2, ...]

    **********Custom ROS Service Description***********
    ob1_arm_control/MatlabIKService

    float32[] pose_target
    float32[] tolerance
    ---
    string result
    float32 error
    float32[] joints
    """
    try:
        rospy.wait_for_service('matlab_ik', timeout=timeout)
    except Exception as err_msg:
        rospy.logwarn(err_msg)
        return 'failed to find service', -1, []

    try:
        request = MatlabIKServiceRequest()
        q = quat_to_list(pose.orientation)
        q = [q[3],q[0],q[1],q[2]]
        rpy = list(transformations.euler_from_quaternion(q))
        xyz = point_to_list(pose.position)
        request.pose_target = xyz + rpy
        request.tolerance = weights
        resp = rospy.ServiceProxy('matlab_ik', MatlabIKService)(request)
        return resp.result, resp.error, resp.joints[:num_joints]
    except rospy.ServiceException as e:
        rospy.logwarn("Service call failed: %s"%e)
        return 'service call failed', -1, []

def relaxedik_service_client(request, timeout=10):
    """
    @brief Client function for relaxed ik service

    @param request: RelaxedIKServiceRequest (read below)

    @return joint target list [angle1,angle2,...]

    Relaxed IK Service Request

    relaxed_ik/EEPoseGoals pose_goals
        std_msgs/Header header
            uint32 seq
            time stamp
            string frame_id
        geometry_msgs/Pose[] ee_poses
            geometry_msgs/Point position
            float64 x
            float64 y
            float64 z
            geometry_msgs/Quaternion orientation
            float64 x
            float64 y
            float64 z
            float64 w

    Relaxed IK Service Response

    relaxed_ik/JointAngles joint_angles
        std_msgs/Header header
            uint32 seq
            time stamp
            string frame_id
        std_msgs/Float32[] angles
            float32 data
    """
    try:
        rospy.wait_for_service('relaxed_ik_service', timeout=timeout)
    except Exception as err_msg:
        rospy.logwarn(err_msg)
        return []

    try:
        resp = rospy.ServiceProxy('relaxed_ik_service', RelaxedIKService)(request)
        return [a.data for a in resp.joint_angles.angles]
    except rospy.ServiceException as e:
        rospy.logwarn("Service call failed: %s"%e)
        return []