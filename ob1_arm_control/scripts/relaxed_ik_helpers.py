from relaxed_ik.msg import EEPoseGoals, JointAngles
from relaxed_ik.srv import RelaxedIKService, RelaxedIKServiceRequest
import rospy

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