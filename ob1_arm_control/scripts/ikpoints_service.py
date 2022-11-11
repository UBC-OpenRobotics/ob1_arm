#!/usr/bin/env python3.8
import rospy
import rospkg
from ikpoints import IKPoints
import time
import numpy as np
from ob1_arm_control.msg import JointTarget
from ob1_arm_control.srv import IKPointsService, IKPointsServiceRequest, IKPointsServiceResponse

def convert_joint_targets_to_Joint_Targets(joint_targets):
    output = []
    if len(joint_targets) > 0:
        if type(joint_targets[0]) is float:
            output = [JointTarget(joint_targets)]
        else:
            for joints in joint_targets:
                output.append(JointTarget(joints))
    return output

def convert_Joint_Targets_to_joint_targets(Joint_Targets):
    output = []
    for joints in Joint_Targets:
        output.append(joints.joint_target)
    return output

def ikpoints_service_client(request):
    """
    @brief Client function for ikpoints service

    @param request: IKPointsServiceRequest (read below)

    @return pose_targets list, joint_targets list, condition
    *lists can be empty , and condition can be False if service call fails

    custom service definition:
    ****IKPointRequest****
        string request
        geometry_msgs/Point point
            float64 x
            float64 y
            float64 z
        uint32 num_pts
        float32 tolerance
        float32 distance

    ****IKPointRequestResponse****
        geometry_msgs/Pose[] pose_targets
        geometry_msgs/Point position
            float64 x
            float64 y
            float64 z
        geometry_msgs/Quaternion orientation
            float64 x
            float64 y
            float64 z
            float64 w
        ob1_arm_control/JointTarget[] joint_targets
            float64[] joint_target
        bool condition
    """
    rospy.wait_for_service('ik_points')
    try:
        resp = rospy.ServiceProxy('ik_points', IKPointsService)(request)
        pose_targets:list = resp.pose_targets
        joint_targets:list = convert_Joint_Targets_to_joint_targets(resp.joint_targets)
        condition:bool = resp.condition
        return pose_targets, joint_targets, condition
    except rospy.ServiceException as e:
        rospy.logwarn("Service call failed: %s"%e)
        return [], [], False

class IKPointsServiceServer:
    ikpoints:IKPoints = None

    def __init__(self):
        rospy.init_node('ikpoints')
        #init and load ikpoints
        rospy.loginfo("==== Loading IK Points ====")
        start = time.time()
        IKPointsServiceServer.ikpoints = IKPoints(rospkg.RosPack().get_path('ob1_arm_control') + "/data/2m_ikpoints_data.json")
        load_time = time.time() - start
        rospy.loginfo("=== Loaded IK points in %d seconds" % load_time)
        rospy.loginfo("==== Done Loading IK Points ====")

        rospy.loginfo("==== Starting IK points service ====")
        rospy.Service('ik_points', IKPointsService, IKPointsServiceServer.handle_ikpoint_request)
        rospy.loginfo("==== Ready for ik point requests ====")

        rospy.spin()

    @staticmethod
    def handle_ikpoint_request(req):
        rospy.loginfo("===ik_points: service call received")
        ikpoints = IKPointsServiceServer.ikpoints
        if type(ikpoints) is IKPoints:
            cmd = req.request
            pt = np.array([req.point.x, req.point.y, req.point.z])
            num_pts = req.num_pts
            if num_pts < 1:
                num_pts = 1
            tolerance = req.tolerance
            if tolerance == 0:
                tolerance = 0.01
            dist = req.distance
            if dist == 0 and 'dist' in cmd:
                rospy.logwarn("Cannot request a distance of 0 with distance related IKPointRequest")
                return IKPointsServiceResponse()

            joint_targets = []
            pose_targets = []
            condition = False
            if cmd == 'get nearest joint targets':
                joint_targets = ikpoints.get_nearest_joint_targets(pt,num_pts)
            elif cmd == 'in range':
                condition = ikpoints.in_range(pt, tolerance)
            elif cmd == 'get dist targets':
                pose_targets, joint_targets = ikpoints.get_dist_targets(pt, dist, tolerance)
            joint_targets = convert_joint_targets_to_Joint_Targets(joint_targets)
            out = IKPointsServiceResponse(pose_targets, joint_targets, condition)
            rospy.loginfo("===ik_points: processed service call")
            return out
        else:
            rospy.logwarn("===ik_points: ikpoints object undefined")
            return IKPointsServiceResponse()

if __name__ == "__main__":
    #start ikpoints service server
    IKPointsServiceServer()
    