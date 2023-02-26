#! /usr/bin/env python
"""
Author: Yousif El-Wishahy
Email: yel.wishahy@gmail.com 

Arm commander class for controlling UBC Open Robotics' Ob1 Arm
"""
###########################################################33
from __future__ import print_function
import sys
import rospy
import moveit_commander
from moveit_commander.conversions import pose_to_list
import numpy as np
import geometry_msgs
from geometry_msgs.msg import PoseStamped, Pose, Point, PointStamped, Quaternion
import geometry_msgs.msg as gm
from moveit_commander import MoveGroupCommander,RobotCommander, PlanningSceneInterface
from shape_msgs.msg import SolidPrimitive
from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import SolidPrimitive
import time
import tf
from tf_helpers import *
from ob1_arm_control.srv import IKPointsServiceRequest
from ikpoints_service import ikpoints_service_client
from relaxed_ik.srv import RelaxedIKService, RelaxedIKServiceRequest
from service_clients import relaxedik_service_client, matlabik_service_client
import pyquaternion as pyq
from functools import cmp_to_key

class ArmCommander:
    _current_plan = None # current joint trajectory plan
    _current_pose_goal : PoseStamped = None #current pose goal 
    _arm_joint_limits = []
    _gripper_joint_limits = []
    _num_joints = 0

    #move group parameters
    _sample_timeout = 1
    _sample_attempts = 5
    _goal_tolerance = 0.01
    _joint_tolerance = 0.001

    #group and link anmes
    ARM_GROUP_NAME = "arm"
    GRIPPER_GROUP_NAME = "gripper"
    EEF_LINK_NAME = "ob1_arm_eef_link"
    GRIPPER_BASE_LINK_NAME = "ob1_arm_gripper_base_link"
    GRIPPER_LCLAW_LINK_NAME = "ob1_arm_lclaw_link"
    GRIPPER_RCLAW_LINK_NAME = "ob1_arm_rclaw_link"
    WORLD_REF_FRAME = "world"
 
    def __init__(self, sample_timeout=1, sample_attempts=5, goal_tolerance=0.01, joint_tolerance=0.001):
        '''
        @brief init arm command object, moveit commander, scene and movegroups for arm and arm gripper
        '''
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node("arm_commander", anonymous=True)
        rospy.loginfo("Initialized arm_commander node")

        self._sample_timeout = sample_timeout
        self._sample_attempts = sample_attempts
        self._goal_tolerance = goal_tolerance
        self._joint_tolerance = joint_tolerance

        self.robot:RobotCommander = moveit_commander.RobotCommander()
        self.scene:PlanningSceneInterface = moveit_commander.PlanningSceneInterface()
        self.tf_listener = tf.TransformListener()
        self.tf_broadcaster = tf.TransformBroadcaster()

        #init arm move group 
        self.arm_mvgroup:MoveGroupCommander = moveit_commander.MoveGroupCommander(self.ARM_GROUP_NAME)
        
        #init gripper move group
        self.gripper_mvgroup:MoveGroupCommander = moveit_commander.MoveGroupCommander(self.GRIPPER_GROUP_NAME)

        self._num_joints = len(self.arm_mvgroup.get_random_joint_values())

        def set_mvgrp_params(mv_grp:MoveGroupCommander):
            mv_grp.set_planning_time(self._sample_timeout)
            mv_grp.set_num_planning_attempts(self._sample_attempts)
            mv_grp.set_goal_tolerance(self._goal_tolerance)
            mv_grp.set_goal_joint_tolerance(self._joint_tolerance)
            rospy.loginfo("==== Initialized move group instance: %s" % mv_grp.get_name())
            rospy.loginfo("==== Sample time out: %s s" % self._sample_timeout)
            rospy.loginfo("==== Sample attempts: %d " % self._sample_attempts)
            rospy.loginfo("==== Goal Tolerance: %f m" % self._goal_tolerance)
            rospy.loginfo("==== Joint Tolerance: %f m" % self._joint_tolerance)
        set_mvgrp_params(self.arm_mvgroup)
        set_mvgrp_params(self.gripper_mvgroup)
        
        rospy.loginfo("==== Planning Reference Frame: %s" % self.arm_mvgroup.get_planning_frame())

        rospy.loginfo("==== Pose Reference Frame: %s" % self.arm_mvgroup.get_pose_reference_frame())

        rospy.loginfo("==== End Effector: %s" % self.arm_mvgroup.get_end_effector_link())

        rospy.loginfo("==== Robot Groups: %s" % self.robot.get_group_names() )

        rospy.loginfo("==== Robot Links ====\n\n%s\n\n" % self.robot.get_link_names())

        rospy.loginfo("==== Robot Joints ====\n\n%s\n\n" % self.robot.get_active_joint_names())

        rospy.loginfo("==== MoveIt Interface Description ====\n\n%s\n" % self.arm_mvgroup.get_interface_description())

        rospy.loginfo("==== Storing Joint Limits")
        for joint_name in self.arm_mvgroup.get_active_joints():
            joint = self.robot.get_joint(joint_name)
            self._arm_joint_limits.append((joint.bounds()[0],joint.bounds()[1]))
        for joint_name in self.gripper_mvgroup.get_active_joints():
            joint = self.robot.get_joint(joint_name)
            self._gripper_joint_limits.append((joint.bounds()[0],joint.bounds()[1]))

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
        if len(self._arm_joint_limits) < 0 :
            return joints
        for i in range(self._num_joints):
            joints[i] = np.clip(joints[i], self._arm_joint_limits[i][0], self._arm_joint_limits[i][1])
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
            if joints[i] < self._arm_joint_limits[i][0] or joints[i] > self._arm_joint_limits[i][1]:
                return False
        return True

    def get_end_effector_pose(self):
        """
        Returns PoseStamped object of end effector of arm move group
        """
        return self.robot.get_link(self.arm_mvgroup.get_end_effector_link()).pose()

    def go_joint(self, joints:list):
        """
        @brief Makes the group's joints go to joint angles specified by goal,
        blocks until goal is reached within tolerance or exit if goal joint
        angles is impossible.

        @param joints: list[float] of joint target angles

        @returns (execution result, joint_target [j0,j1,j2,...] | None)
        """
        if type(joints) is not list:
            joints = convert_to_list(joints)

        if len(joints) != self._num_joints:
            raise ValueError("joints list has incorrect length (!=%d)"%self._num_joints)

        self.arm_mvgroup.set_joint_value_target(joints)
        result = self.arm_mvgroup.go()
        self.arm_mvgroup.stop()
        return result, joints
    
    def go_position(self, position_goal):
        """
        @brief makes the end effector of the arm go to a cartesian position (x,y,z)
        The position should be in the reference frame of the base link

        @param position_goal: can be list of [x,y,z] a Point, Pose, or PoseStamped objects
        The position will be extracted from any of these data types.

        @returns (execution result, position target [x,y,z] )
        """

        if type(position_goal) is not list:
            position_goal = convert_to_list(position_goal)
            if len(position_goal) != 3:
                raise ValueError("Position goal list has invalid length (!=3)")

        self.arm_mvgroup.set_position_target(position_goal, self.GRIPPER_BASE_LINK_NAME)
        res = self.arm_mvgroup.go()
        self.arm_mvgroup.stop()
    
        return res, position_goal

    def go_position_ikpoints(self, position_goal):
        """
        @brief makes the end effector of the arm go to a cartesian position (x,y,z)
        Finds optimal joint target based on ik points data base
        The position should be in the reference frame of the base link

        @param position_goal: can be list of [x,y,z] a Point, Pose, or PoseStamped objects
        The position will be extracted from any of these data types.

        @returns (execution result, position target [x,y,z], joint_target [j0,j1,j2,...])
        """

        #handle different kind of inputs
        if type(position_goal) is not list:
            position_goal = convert_to_list(position_goal)
            if len(position_goal) != 3:
                raise ValueError("Position goal list has invalid length (!=3)")

        req = IKPointsServiceRequest()
        req.request = 'get nearest joint targets'
        req.pose = Pose(Point(position_goal[0],position_goal[1],position_goal[2]),Quaternion())
        joint_target = ikpoints_service_client(req)[1]
        res, _ = self.go_joint(joint_target)
    
        return res, position_goal, joint_target
        
    def go_pose(self, pose_goal:PoseStamped):
        '''
        @brief Makes the end effector of the arm go to a pose, 
        blocks until either inverse kinematics fails 
        to find path or it has reached the goal within tolerace.

        @param pose_goal : geometry_msgs\Pose contains desired 
        position and orientation for gripper

        if pose_goal param is not passed, random pose goal will be generated

        @returns (execution result, PoseStamped target)
        '''

        if type(pose_goal) is not PoseStamped:
            raise ValueError("Input goal is not a PoseStamped")
        if self.robot.get_planning_frame() != pose_goal.header.frame_id:
            raise ValueError("Incorrect planning frame for pose goal, got %s instead of %s \n" \
                    %(self._current_pose_goal.header.frame_id,self.robot.get_planning_frame()))

        self._current_pose_goal = pose_goal

        self.arm_mvgroup.set_pose_target(pose_goal)
        res = self.arm_mvgroup.go()
        self.arm_mvgroup.stop()
        self.arm_mvgroup.clear_pose_targets()

        return res, pose_goal

    def go_position_matlabik(self, position_goal:Point):
        if type(position_goal) is not Point:
            raise ValueError("Input goal is not a Point")
        pose = Pose()
        pose.position = position_goal

        result_str, error, joints = matlabik_service_client(pose, [0,0,0,1,1,1])
        rospy.loginfo("Matlab IK response: %s, error: %.6f" %(result_str,error))
        if 'success' or 'best available' in result_str:
            rospy.loginfo(joints)
            if self._check_joint_limits(joints):
                res,_ = self.go_joint(joints)
                return res, position_goal, joints
        return False, position_goal, None

    def go_pose_matlabik(self, pose_goal:PoseStamped = None, weights=[0.25,0.25,0.25,1,1,1]):
        if type(pose_goal) is not PoseStamped:
            raise ValueError("Input goal is not a PoseStamped")
        if self.robot.get_planning_frame() != pose_goal.header.frame_id:
            print("Incorrect planning frame for pose goal, got %s instead of %s \n" \
                    %(self._current_pose_goal.header.frame_id,self.robot.get_planning_frame()))
            self._current_pose_goal = None
            return False, None
        self._current_pose_goal = pose_goal

        result_str, error, joints = matlabik_service_client(pose_goal.pose, weights)
        rospy.loginfo("Matlab IK response: %s, error: %.6f" %(result_str,error))
        if 'success' or 'best available' in result_str:
            if self._check_joint_limits(joints):
                res,_ = self.go_joint(joints)
                return res, pose_goal, joints
        return False, pose_goal, None

    def go_pose_relaxedik(self, pose_goal:PoseStamped = None):
        if pose_goal is None:
            pose_goal = self.arm_mvgroup.get_random_pose()
        if type(pose_goal) is not PoseStamped:
            raise ValueError("Input goal is not a PoseStamped")
        if self.robot.get_planning_frame() != pose_goal.header.frame_id:
            print("Incorrect planning frame for pose goal, got %s instead of %s \n" \
                    %(self._current_pose_goal.header.frame_id,self.robot.get_planning_frame()))
            self._current_pose_goal = None
            return False, None
        self._current_pose_goal = pose_goal

        req = RelaxedIKServiceRequest()
        req.pose_goals.header = pose_goal.header
        req.pose_goals.ee_poses = [pose_goal.pose]
        joints = relaxedik_service_client(req)
        if len(joints) != self._num_joints:
            res = False
        else:
            res, _ = self.go_joint(joints)
        return res, pose_goal, joints
    
    @staticmethod
    def get_indices_optimal_poses(goal, pose_list:list, n:int, comparator_id=2):
        """
        @brief Return indices of up to n optimal poses from the pose list passes.
        Optimal pose is defined as the closest pose to the pose_goal by angular magnitude.

        @param pose_goal: geometry_msgs/Pose object
        @param pose_list: list of poses to filter
        @param n: amount of filtered poses to return (e.g. if 1 only return the closest, if 2 return the 1st and 2nd closest)

        @return a list of ordered integer indices
        """
        if type(goal) is PoseStamped:
            q = goal.pose.orientation
        elif type(goal) is Pose:
            q = goal.orientation
        elif type(goal) is Quaternion:
            q = goal
        else:
            raise ValueError("parameter is not a valid point; needs to be geometry_msgs/Pose)")
        if n > len(pose_list):
            n = len(pose_list)

        QuaternionComparators.set_ref_quaternion(q)
        cmp = QuaternionComparators.get_comparator(comparator_id)

        sortable = [(i, pose_list[i].orientation) for i in range(len(pose_list))]
        sortable = sorted(sortable, key=cmp_to_key(lambda item1, item2: cmp(item1[1], item2[1])))

        return [s[0] for s in sortable][0:n]
    
    def rotate_gripper_axis(self, w:float, timeout=5):
        """
        @brief brute force method to rotate gripper around current axis to match a certain orientation w value
        **the logic is kinda bad**

        @param w : float for the w value of a quaternion (this represents amount of rotation about current orientation axis)
        @param timeout: float timeout value for read loop
        """
        joints = self.arm_mvgroup.get_current_joint_values()
        min, max = self._arm_joint_limits[-1]
        joints[-1]  = min
        self.go_joint(joints)
        joints[-1] = max
        self.arm_mvgroup.go(joints, wait=False)
        loop_start_time = rospy.get_time()
        d_curr = 1000
        val = 0
        while rospy.get_time() < loop_start_time + timeout:
            w_curr = self.get_end_effector_pose().pose.orientation.w
            if np.abs(w_curr-w) < d_curr:
                d_curr = np.abs(w_curr-w)
                val = self.arm_mvgroup.get_current_joint_values()[-1]
        joints[-1] = val
        rospy.loginfo("going to %s with min error %.5f" % (joints, d_curr))
        self.go_joint(joints)

    def go_pose_ikpoints(self, pose_goal:PoseStamped=None):
        '''
        @brief Approximately goes to the desired pose_goal. This is a blocking method

        procedure: queries ikpoints database, finds closest reachable poses up to tolerance distance, 
        and sorts these poses based on orientation angular magnitude comparisons

        @param pose_goal : geometry_msgs\Pose contains desired position and orientation for the end effector link.

        if pose_goal param is not passed, random pose goal will be generated.

        @returns (execution result, original PoseStamped target, actual joint_target [j0,j1,j2,...])
        '''
        if pose_goal is None:
            pose_goal = self.arm_mvgroup.get_random_pose()
        if type(pose_goal) is not PoseStamped:
            raise ValueError("Input goal is not a PoseStamped")
        if self.robot.get_planning_frame() != pose_goal.header.frame_id:
            print("Incorrect planning frame for pose goal, got %s instead of %s \n" \
                    %(self._current_pose_goal.header.frame_id,self.robot.get_planning_frame()))
            self._current_pose_goal = None
            return False, None, None
        self._current_pose_goal = pose_goal

        req = IKPointsServiceRequest()
        req.request = 'get up to dist targets'
        req.pose = pose_goal.pose
        req.distance = self._goal_tolerance
        pose_targets , joint_targets, _ = ikpoints_service_client(req)
        indices = self.get_indices_optimal_poses(pose_goal.pose, pose_targets, 1)

        for i in indices:
            joints = joint_targets[i]
            res, _ = self.go_joint(joints)
            if res:
                self.rotate_gripper_axis(pose_goal.pose.orientation.w)
                return res, pose_goal, joints
        return False, pose_goal, None

    def get_object(self, object_id:str):
        """
        @brief Get collision object from the planning scene based on id string

        @return Moveit_msgs/CollisionObject if found | None if not found
        """
        objs = self.scene.get_objects()
        for o in list(objs.items()):
            obj:CollisionObject = o[1]
            # shape:SolidPrimitive = obj.primitives[0]
            if obj.id == object_id:
                return obj
        print('could not find obj in %s' %objs)
        return None

    def close_gripper(self):
        """
        @brief close gripper to previously defined 'close' joint values
        """
        joints_close = list(self.gripper_mvgroup.get_named_target_values("close").values())
        return self.gripper_mvgroup.go(joints_close)

    def open_gripper(self):
        """
        @brief open gripper to previously defined 'open' joint values
        """
        joints_close = list(self.gripper_mvgroup.get_named_target_values("open").values())
        return self.gripper_mvgroup.go(joints_close)

    def is_object_attached(self, object_id:str):
        """
        @brief check if object with id object_id is attached to the arm/gripper in the planning scene
        """
        res:bool = False 
        if self.scene.get_attached_objects([object_id]):
            #empty dictionaries eval to false in python
            res = True
        return res

    def attach_object(self, object:CollisionObject):
        """
        @brief attach CollisionObject object to the gripper in the planning scene
        """
        touch_links = self.robot.get_link_names(group=self.GRIPPER_GROUP_NAME)
        self.scene.attach_box(self.EEF_LINK_NAME, object.id, touch_links=touch_links)
        time.sleep(2) #delay for scene update
        return self.is_object_attached(object.id)
    
    def detach_object(self, object:CollisionObject):
        """
        @brief detach CollisionObject object from the gripper in the planning scene
        """
        self.scene.remove_attached_object(self.EEF_LINK_NAME, name=object.id)
        time.sleep(2) #delay for scene update
        return not self.is_object_attached(object.id)

    def pick_object(self, object:CollisionObject):
        def close_gripper_around_object():
            self.gripper_mvgroup.set_planning_time(0.1)
            self.gripper_mvgroup.set_num_planning_attempts(1)
            joints_open = list(self.gripper_mvgroup.get_named_target_values("open").values())
            joints_close = list(self.gripper_mvgroup.get_named_target_values("close").values())
            js_0 = np.linspace(joints_close[0], joints_open[0], 10)
            js_1 = np.linspace(joints_close[1], joints_open[1], 10)
            for a0 in js_0:
                for a1 in js_1:
                    self.gripper_mvgroup.set_joint_value_target([a0,a1])
                    plan = self.gripper_mvgroup.plan()
                    if plan[0]:
                        if self.gripper_mvgroup.go():
                            return True
            return False
        
        print("Picking object: %s" % object.id)
        self.open_gripper()

        res, _ = self.go_position(object.pose.position)
        if res:
            res = close_gripper_around_object()
            if res:
                res = self.attach_object(object)
        return res

    def pick_object_ikpoints(self, object:CollisionObject, attempts=100, pick_tolerance=0.01):
        def close_gripper_around_object():
            """
            @brief close gripper around object by test collision state of joint values
            """
            joints_open = list(self.gripper_mvgroup.get_named_target_values("open").values())
            joints_close = list(self.gripper_mvgroup.get_named_target_values("close").values())
            js_0 = np.linspace(joints_close[0], joints_open[0], 10)
            js_1 = np.linspace(joints_close[1], joints_open[1], 10)
            for a0 in js_0:
                for a1 in js_1:
                    self.gripper_mvgroup.set_joint_value_target([a0,a1])
                    plan = self.gripper_mvgroup.plan()
                    if plan[0]:
                        if self.gripper_mvgroup.go():
                            return True
            return False
        
        print("Picking object: %s" % object.id)
        self.open_gripper()

        req = IKPointsServiceRequest()
        req.request = "in range"
        req.pose = object.pose
        req.tolerance = self._goal_tolerance
        if not ikpoints_service_client(req)[2]:
            rospy.logwarn("Object outside of reachable space.")
            return False

        # req = IKPointsServiceRequest()
        # req.request = 'get nearest joint targets'
        # req.pose = object.pose
        # req.tolerance = pick_tolerance
        # req.num_pts = 100
        # joint_targets = ikpoints_service_client(req)[1]

        current_dist = 0.0
        max_dist = 0.1
        tolerance = 0.005
        increment = pick_tolerance
        res = False

        while current_dist < max_dist:
            req = IKPointsServiceRequest()
            req.request = 'get dist joint targets'
            req.pose = object.pose
            req.tolerance = tolerance
            req.distance = current_dist
            joint_targets = ikpoints_service_client(req)[1][0:attempts]

            for joints in joint_targets:
                res, _ = self.go_joint(joints)
                if res:
                    break
            if res:
                break
            current_dist+=increment

        if res:
            res = close_gripper_around_object()
        if res:
            res = self.attach_object(object)
        return res
    
    def place_object(self, object:CollisionObject, place_pose:PoseStamped, attempts=1000, place_tolerance=0.05):
        """
        @brief place function to place an object in the planning scene with a desired pose

        @param object: CollisionObject in planning scene (make sure it is updated)
        This should already be attached to arm or this fucntion will return false

        @param place_pose: desired pose to place the object in
        Can be in any ref frame (this function handles transformations as long as they exist in /tf)

        @return bool: indicates successful execution of place operation
        """
        if type(object) != CollisionObject:
            raise ValueError("object type is not CollisionObject. It is %s" % type(object))
        if not self.is_object_attached(object.id):
            rospy.logwarn("Cannot place object. Object with id %s is NOT attached to arm in planning scene." % object.id)
            return False
        rospy.loginfo("picking object")
        req = IKPointsServiceRequest()
        req.request = "in range"
        req.pose = object.pose
        req.tolerance = self._goal_tolerance
        if not ikpoints_service_client(req)[2]:
            rospy.logwarn("Object outside of reachable space.")
            return False

        if place_pose.header.frame_id != self.WORLD_REF_FRAME:
            rospy.loginfo("Converting target place pose to world frame.")
            place_pose = self.tf_listener.transformPose("/world",place_pose)

        #first we should get the tf from object --> eef : t_obj_eef (numpy matrix 4x4)
        t_obj_w = np.linalg.inv(pose_to_mat(object.pose))
        t_w_eef = pose_to_mat(self.get_end_effector_pose().pose)
        #this should be constant until the detach operation
        t_obj_eef = np.dot(t_obj_w, t_w_eef)

        #use t_obj_eef to get target end effector pose
        t_w_tobj = pose_to_mat(place_pose.pose) # world -> place object pose 
        t_w_teef = np.dot(t_w_tobj,t_obj_eef) # world -> place object end effector pose
        pose_goal = mat_to_pose(t_w_teef) #target pose goal in world frame
        self._current_pose_goal = pose_goal

        req = IKPointsServiceRequest()
        req.request = 'get up to dist targets'
        req.pose = pose_goal
        req.distance = place_tolerance
        pose_targets , joint_targets, _ = ikpoints_service_client(req)

        indices = self.get_indices_optimal_poses(pose_goal, pose_targets, attempts)
        rospy.loginfo("got %d inidces" % len(indices))

        res = False
        self.arm_mvgroup.clear_pose_targets()
        for i in indices:
            joints = joint_targets[i]
            res, _ = self.go_joint(joints)
            if res:
                self.rotate_gripper_axis(pose_goal.orientation.w)
                break

        if res:
            res = self.detach_object(object)
        if res:
            self.open_gripper()
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
        req.pose = object.pose
        req.tolerance = 0.01
        if not ikpoints_service_client(req)[2]:
            return False, object.pose, None

        pt_tclaw = np.array([-0.05,-0.07,0])
        pose_tclaw = Pose(Point(*tuple(pt_tclaw)), Quaternion(w=1))
        rclaw_tclaw_mat44 = pose_to_mat(pose_tclaw)
        t, r = self.tf_listener.lookupTransform('/'+self.GRIPPER_BASE_LINK_NAME, '/'+self.GRIPPER_RCLAW_LINK_NAME, rospy.Time(0))
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
        req.pose = object.pose
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
