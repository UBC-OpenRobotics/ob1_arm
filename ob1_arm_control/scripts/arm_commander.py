#!/usr/bin/env python3.8
from __future__ import print_function
import sys
import rospy
import moveit_commander
from moveit_commander.conversions import pose_to_list
import numpy as np
from math import pi, tau, dist, fabs, cos
import geometry_msgs
from geometry_msgs.msg import PoseStamped, Pose, Point, PointStamped, Quaternion
import geometry_msgs.msg as gm
from moveit_commander import MoveGroupCommander,RobotCommander, PlanningSceneInterface
from shape_msgs.msg import SolidPrimitive
from moveit_commander.planning_scene_interface import CollisionObject
import rospkg
from ikpoints import IKPoints
import kinpy as kp
from kinpy.chain import SerialChain
import time
import tf
from tf_helpers import *
from ob1_arm_control.srv import IKPointsServiceRequest
from ikpoints_service import ikpoints_service_client

# Author: Yousif El-Wishahy

##############################
# arm command class to be called by robot loop
##############################
class ArmCommander:
    _current_plan = None # current joint trajectory plan
    _current_pose_goal : PoseStamped = None #current pose goal 
    _sample_time_out   = 5             # time out in seconds for Inverse Kinematic search
    _sample_attempts   = 5             # num of planning attempts 
    _goal_tolerance    = 0.001
    _joint_limits = []
    _num_joints = 0
    _gripper_offset = [0,-0.155,0.08285]

    #group and link anmes
    ARM_GROUP_NAME = "arm"
    GRIPPER_GROUP_NAME = "gripper"
    GRIPPER_LINK_NAME = "ob1_arm_gripper_base_link"
    GRIPPER_LCLAW_LINK_NAME = "ob1_arm_lclaw_link"
    GRIPPER_RCLAW_LINK_NAME = "ob1_arm_rclaw_link"
    WORLD_REF_FRAME = "world"

    URDF_PATH = rospkg.RosPack().get_path('ob1_arm_description') + "/urdf/main.urdf"
    kinpy_arm:SerialChain = None
 
    def __init__(self, sample_time_out=5, sample_attempts=5, goal_tolerance=0.05):
        '''
        @brief init arm command object, moveit commander, scene and movegroups for arm and arm gripper
        '''
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node("arm_commander", anonymous=True)
        rospy.loginfo("Initialized arm_commander node")

        self.robot:RobotCommander = moveit_commander.RobotCommander()
        self.scene:PlanningSceneInterface = moveit_commander.PlanningSceneInterface()
        self.tf_listener = tf.TransformListener()
        self.tf_broadcaster = tf.TransformBroadcaster()

        #init arm move group 
        self.arm_mvgroup:MoveGroupCommander = moveit_commander.MoveGroupCommander(self.ARM_GROUP_NAME)
        self.arm_mvgroup.set_planning_time(sample_time_out)
        self.arm_mvgroup.set_num_planning_attempts(sample_attempts)
        self.arm_mvgroup.set_goal_tolerance(goal_tolerance)
        self.arm_mvgroup.set_max_velocity_scaling_factor(1)
        self.arm_mvgroup.set_max_acceleration_scaling_factor(0.3)

        self._num_joints = len(self.arm_mvgroup.get_random_joint_values())
        self._goal_tolerance = goal_tolerance
        self._sample_attempts = sample_attempts
        self._sample_time_out = sample_time_out

        self.kinpy_arm = kp.build_serial_chain_from_urdf(
            open(self.URDF_PATH).read(),
            root_link_name="ob1_arm_base_link",
            end_link_name="ob1_arm_gripper_base_link"
        )
        rospy.loginfo("==== Initialized kinpy arm serial chain")

        rospy.loginfo("==== Initialized arm move group")
        rospy.loginfo("==== Sample time out: %s s" % self._sample_time_out)
        rospy.loginfo("==== Sample attempts: %d " % self._sample_attempts)
        rospy.loginfo("==== Goal Tolerance: %f m" % self._goal_tolerance)

        self.gripper_mvgroup:MoveGroupCommander = moveit_commander.MoveGroupCommander(self.GRIPPER_GROUP_NAME)
        
        #debug print
        rospy.loginfo("==== Planning Reference Frame: %s" % self.arm_mvgroup.get_planning_frame())

        rospy.loginfo("==== Pose Reference Frame: %s" % self.arm_mvgroup.get_pose_reference_frame())

        rospy.loginfo("==== End Effector: %s" % self.arm_mvgroup.get_end_effector_link())

        rospy.loginfo("==== Robot Groups: %s" % self.robot.get_group_names() )

        rospy.loginfo("==== Robot Links ====\n\n%s\n\n" % self.robot.get_link_names())

        rospy.loginfo("==== Robot Joints ====\n\n%s\n\n" % self.robot.get_active_joint_names())

        # print("============ Printing robot state")
        # print(self.robot.get_current_state())

        rospy.loginfo("==== MoveIt Interface Description ====\n\n%s\n" % self.arm_mvgroup.get_interface_description())

        rospy.loginfo("==== Storing Joint Limits")
        for joint_name in self.arm_mvgroup.get_active_joints():
            joint = self.robot.get_joint(joint_name)
            self._joint_limits.append((joint.bounds()[0],joint.bounds()[1]))
        self.register_gripper_edge_states()

    def _check_pose_goal(self,pose):
        if type(pose) is PoseStamped:
            if pose.header.frame_id == self.arm_mvgroup.get_planning_frame():
                return True
        return False

    def _enforce_joint_limits(self,joints):
        """
        @brief helper function to ensure joint limits

        @param joints, list[floats] of joint values

        @return list of joint values clamped to their limits (as defined by the urdf)
        """
        if len(self._joint_limits) < 0 :
            return joints
        for i in range(self._num_joints):
            joints[i] = np.clip(joints[i], self._joint_limits[i][0], self._joint_limits[i][1])
            print(joints[i])
        return joints

    def _check_joint_limits(self,joints):
        """
        @brief check that joint values do not exceed limits

        @param joints, list[floats] of joint values

        @return bool
        """
        if len(joints) != self._num_joints:
            return False
        for i in range(self._num_joints):
            if joints[i] < self._joint_limits[i][0] or joints[i] > self._joint_limits[i][1]:
                return False
        return True

    def register_gripper_edge_states(self):
        joint_vals_min = [0]
        joint_vals_max = [0]
        for joint_name in self.gripper_mvgroup.get_joints():
            joint = self.robot.get_joint(joint_name)
            joint_vals_min.append(joint.bounds()[0])
            joint_vals_max.append(joint.bounds()[1])

        #todo: test these remembered joint states
        self.gripper_mvgroup.remember_joint_values("open", values=joint_vals_max)
        self.gripper_mvgroup.remember_joint_values("close", values=joint_vals_min)

    def get_end_effector_pose(self):
        """
        Returns PoseStamped object of end effector of arm move group
        """
        return self.robot.get_link(self.arm_mvgroup.get_end_effector_link()).pose()

    def go_joint(self, joints:list=None):
        """
        @brief Makes the group's joints go to joint angles specified by goal,
        blocks until goal is reached within tolerance or exit if goal joint
        angles is impossible.

        @param group (MoveGroupCommander) : Move group object , either arm or gripper

        @param joints: list[float] of joint target angles

        @returns (command result, joint_target [j0,j1,j2,...])
        """
        if type(joints) is not list:
            if type(joints) is np.ndarray:
                joints = joints.tolist()
            elif type(joints) is tuple:
                joints = list(joints)
        if type(joints) is not list or len(joints) != self._num_joints: 
            print('getting random joint values due to joints type = %s' % type(joints))
            joints:list = self.arm_mvgroup.get_random_joint_values()

        self.arm_mvgroup.clear_pose_targets()
        self.arm_mvgroup.set_joint_value_target(joints)
        plan = self.arm_mvgroup.plan()
        result:bool = plan[0]

        if plan[0]:
            result = self.arm_mvgroup.go()
            self.arm_mvgroup.stop()
        self._current_plan = plan
        return result, joints
    
    def go_position(self, position_goal=None):
        """
        @brief makes the end effector of the arm go to a cartesian position (x,y,z)
        The position should be in the reference frame of the base link

        @param position_goal: can be list of [x,y,z] a Point, Pose, or PoseStamped objects
        The position will be extracted from any of these data types.

        @returns (execution result, position target [x,y,z] )
        """

        #handle different kind of inputs
        if position_goal is None:
            position_goal = convert_to_list(self.arm_mvgroup.get_random_pose().pose.position)
        elif type(position_goal) is not list:
            position_goal = convert_to_list(position_goal)

        #plan to original position goal
        self.arm_mvgroup.clear_pose_targets()
        self.arm_mvgroup.set_position_target(position_goal, self.GRIPPER_LINK_NAME)
        plan = self.arm_mvgroup.plan()
        self._current_plan = plan

        if plan[0]:
            res = self.arm_mvgroup.go()
            self.arm_mvgroup.stop()
            self.arm_mvgroup.clear_pose_targets()
    
        return res, position_goal

    def go_position_ikpoints(self, position_goal=None):
        """
        @brief makes the end effector of the arm go to a cartesian position (x,y,z)
        Finds optimal joint target based on ik points data base
        The position should be in the reference frame of the base link

        @param position_goal: can be list of [x,y,z] a Point, Pose, or PoseStamped objects
        The position will be extracted from any of these data types.
        **IF position_goal is None, a random position_goal will be selected and the nearest ikpoint will be matched to it

        @returns (execution result, position target [x,y,z], joint_target [j0,j1,j2,...])
        """

        #handle different kind of inputs
        if position_goal is None:
            position_goal = convert_to_list(self.arm_mvgroup.get_random_pose().pose.position)
        elif type(position_goal) is not list:
            position_goal = convert_to_list(position_goal)

        req = IKPointsServiceRequest()
        req.request = 'get nearest joint targets'
        req.point = Point(position_goal[0],position_goal[1],position_goal[2])
        joint_target = ikpoints_service_client(req)[1]
        res, _ = self.go_joint(joint_target)
    
        return res, position_goal, joint_target
        
    def go_pose(self, pose_goal:PoseStamped=None):
        '''
        @brief Makes the end effector of the arm go to a pose, 
        blocks until either inverse kinematics fails 
        to find path or it has reached the goal within tolerace.

        @param pose_goal : geometry_msgs\Pose contains desired 
        position and orientation for gripper

        if pose_goal param is not passed, random pose goal will be generated

        @returns (execution result, PoseStamped target)
        '''

        if pose_goal == None:
            pose_goal = self.arm_mvgroup.get_random_pose()
        if type(pose_goal) is not PoseStamped:
            raise ValueError("Input goal is not a PoseStamped")
        if self.robot.get_planning_frame() != pose_goal.header.frame_id:
            print("Incorrect planning frame for pose goal, got %s instead of %s \n" \
                    %(self._current_pose_goal.header.frame_id,self.robot.get_planning_frame()))
            self._current_pose_goal = None
            return False, None
        self._current_pose_goal = pose_goal

        plan = self.arm_mvgroup.plan(self._current_pose_goal.pose)
        self._current_plan = plan
        res:bool = plan[0]

        if plan[0]:
            res = self.arm_mvgroup.go()
            self.arm_mvgroup.stop()
            self.arm_mvgroup.clear_pose_targets()

        return res, pose_goal

    def close_gripper(self):
        joints = self.gripper_mvgroup.get_remembered_joint_values("close")
        return self.gripper_mvgroup.go(joints)
    
    def pick(self, object:CollisionObject):
        res, pos_target, joint_target = self.go_position_ikpoints(object.pose.position)
        if res:
            res = self.close_gripper()
        return res

    def go_scene_object(self, object:CollisionObject, claw_search_tolerance=0.05, orientation_search_tolerance = 0.5):
        """
        @brief makes the end effector of the arm go to the proximity of a scene object
        Finds optimal joint target based on ik points data base
        The position of the object should be in the reference frame of the base link
        Proximity is based on the closest reachable ikpoint to the scene object position

        @param object: scene object with type moveit_commander.planning_scene_interface.CollisionObject

        @param(optional,defuault=10) attempts: number of different points to attempt near scene object

        @returns (execution result, object pose, successful_joint_target [j0,j1,j2,...] | None)
        """

        #add object transform to transform topic
        broadcast_pose(self.tf_broadcaster, object.pose, object.id, object.header.frame_id)
        pt_obj = np.array(convert_to_list(object.pose.position)) #object point (frame: world)

        #TO DO : better check for in range (since we do more complex searches)
        req = IKPointsServiceRequest()
        req.request = "in range"
        req.point = object.pose.position
        req.tolerance = 0.01
        if not ikpoints_service_client(req)[2]:
            return False, object.pose, None

        pt_tclaw = np.array([-0.05,-0.07,0])
        pose_tclaw = Pose(Point(*tuple(pt_tclaw)), Quaternion(w=1))
        rclaw_tclaw_mat44 = pose_to_mat(pose_tclaw)
        t, r = self.tf_listener.lookupTransform('/'+self.GRIPPER_LINK_NAME, '/'+self.GRIPPER_RCLAW_LINK_NAME, rospy.Time(0))
        eef_rclaw_mat44 = t_r_to_mat(t,r)
        #need this tansformation for later
        w_eef_mat44 = pose_to_mat(self.get_end_effector_pose().pose)
        eef_tclaw_mat44 = np.dot(eef_rclaw_mat44,rclaw_tclaw_mat44)

        w_tclaw_pose =  mat_to_pose(np.dot(w_eef_mat44,eef_tclaw_mat44))
        broadcast_pose(self.tf_broadcaster, w_tclaw_pose, 'og_pose_tclaw', 'world')


        #find all points a 'claw offset' distance away from the centre of the object
        claw_offset = np.linalg.norm(pt_tclaw)
        start = time.time()
        req = IKPointsServiceRequest()
        req.request = 'get dist targets'
        req.point = object.pose.position
        req.distance = claw_offset
        req.tolerance = 0.05
        pose_targets, joint_targets, _ = ikpoints_service_client(req)
        search_dur = time.time()- start
        print("found %d pose targets matching distance %.5f in %.5f seconds" % (len(pose_targets),claw_offset,search_dur))
        assert len(joint_targets) == len(pose_targets), "pose targets and joint targets list not same size"
        size = len(pose_targets)

        #filter through poses by calculation quaternion absolute distances
        #this roughly determines if pose orientation is facing object
        start = time.time()
        f_ikpoints = []
        for i in range(size):
            pose_target:Pose = pose_targets[i]

            w_eef_mat44 = pose_to_mat(pose_target)
            w_tclaw_mat44 = np.dot(w_eef_mat44, eef_tclaw_mat44)
            pose_tclaw = mat_to_pose(w_tclaw_mat44)
            pt_tclaw = np.array(convert_to_list(pose_tclaw.position))
            d_tclaw = np.linalg.norm(pt_obj-pt_tclaw)
            if d_tclaw > claw_search_tolerance:
                continue
            broadcast_pose(self.tf_broadcaster, pose_tclaw, 'pose_tclaw', 'world')

            f_ikpoints.append((d_tclaw,pose_targets[i],joint_targets[i]))

        del(pose_targets)
        del(joint_targets)

        from functools import cmp_to_key
        sorted(f_ikpoints, key=cmp_to_key(lambda t1, t2: t2[0]-t1[0]))
        joint_targets = [t[2] for t in f_ikpoints]
        pose_targets = [t[1] for t in f_ikpoints]
        print(f_ikpoints[0][0], f_ikpoints[-1][0])

        filter_dur = time.time()- start
        print("filtered %d pose targets in %.5f seconds" % (size-len(joint_targets), filter_dur))
        print("%d joint targets to test" % len(joint_targets))

        for joints in joint_targets:
            res, _ = self.go_joint(joints)
            if res:
                return res, object.pose, joints
        return False, object.pose, None

def convert_to_list(obj):
    """
    @brief helper function to convert geometry messages to list [x,y,z]

    returns none if invalid input object
    """
    if type(obj) is gm.Point:
        return [obj.x,obj.y,obj.z]
    if type(obj) is gm.Quaternion:
        return [obj.x,obj.y,obj.z,obj.w]
    elif type(obj) is gm.Pose:
        return [ obj.position.x,
                 obj.position.y,
                 obj.position.z ]
    elif type(obj) is gm.PoseStamped:
        return [ obj.pose.position.x,
                 obj.pose.position.y,
                 obj.pose.position.z ]
    elif type(obj) is np.ndarray:
        return obj.tolist()
    elif type(obj) is list:
        return obj
    else:
        print(obj)
        raise ValueError("Invalid input to convert_to_list funtion")
