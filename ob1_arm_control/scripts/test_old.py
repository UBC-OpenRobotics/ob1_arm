#!/usr/bin/env python3.8
from distutils.util import rfc822_escape
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi, tau, dist, fabs, cos

HOR = '-'*20

# Robot commander
robot = moveit_commander.RobotCommander()

# move group commander
group_name = "arm"
move_group = moveit_commander.MoveGroupCommander(group_name)

# Scene Planning
scene = moveit_commander.PlanningSceneInterface()

# We can get the name of the reference frame for this robot:
planning_frame = move_group.get_planning_frame()
print("============ Planning frame: %s" % planning_frame)

# We can also print the name of the end-effector link for this group:
eef_link = move_group.get_end_effector_link()
print("============ End effector link: %s" % eef_link)

# We can get a list of all the groups in the robot:
group_names = robot.get_group_names()
print("============ Available Planning Groups:", robot.get_group_names())

# Sometimes for debugging it is useful to print the entire state of the
# robot:
print("============ Printing robot state")
print(robot.get_current_state())
print("")

rospy.init_node('armplanner')

for i in range(10):
    pose = move_group.get_random_pose()
    # pose_goal = geometry_msgs.msg.Pose()
    # pose_goal.orientation.w = 1.0
    # pose_goal.position.x = 0.4
    # pose_goal.position.y = 0.1
    # pose_goal.position.z = 0.4

    # move_group.set_pose_target(pose_goal)
    move_group.set_pose_target(pose)

    plan = move_group.go(wait=True)
    # Calling `stop()` ensures that there is no residual movement
    move_group.stop()
    # It is always good to clear your targets after planning with poses.
    # Note: there is no equivalent function for clear_joint_value_targets()
    move_group.clear_pose_targets()
    if plan: break
# move_group.execute(plan, wait=True)

for i in range(10):
    joints = move_group.get_random_joint_values()
    move_group.set_joint_value_target(joints)
    plan = move_group.go(wait=True)
    move_group.stop()
    move_group.clear_pose_targets()
    if plan: break
# plan = move_group.plan(joints)
# move_group.execute(plan, wait=True)

print(HOR,)
print(f'sent random joint states: {plan}')
rospy.sleep(5)

named_pose = 'up'
print(HOR,)
print(f'Going to {named_pose}')
move_group.set_named_target(named_pose)
move_group.go()
move_group.stop()
move_group.clear_pose_targets()
print(HOR,)
print(f'Went to {named_pose}')
rospy.sleep(5)

named_pose = 'folded'
print(HOR,)
print(f'Going to {named_pose}')
move_group.set_named_target(named_pose)
move_group.go()
move_group.stop()
move_group.clear_pose_targets()
print(HOR,)
print(f'Went to {named_pose}')
rospy.sleep(5)


ref_frame = move_group.get_pose_reference_frame
print(ref_frame)

# Add box
box_pose = geometry_msgs.msg.PoseStamped()
box_pose.header.frame_id = 'shoulder_link'
box_pose.pose.orientation.w = 1.0
box_pose.pose.position.z = 0.11  # above the panda_hand frame
box_pose.pose.position.x = 0.20
box_pose.pose.position.y = 0.20
box_name = "box"
scene.add_box(box_name, box_pose, size=(0.075, 0.075, 0.075))
rospy.sleep(10)
scene.remove_world_object(box_name)

############################
# Arm and gripper grasp object test
# the following code is adapated from 
# https://github.com/AIWintermuteAI/ros-moveit-arm/blob/master/my_arm_xacro/pick/pick.py
# with some minor changes
###########################

def degrees_to_radians(joint_angles):
    for i in range(len(joint_angles)):
        joint_angles[i] = joint_angles[i] * np.pi/180.0
    return joint_angles


# Get the gripper posture as a JointTrajectory
def make_gripper_posture(arm_commander : ArmCommander, joint_positions):
    # Initialize the joint trajectory for the gripper joints
    t = JointTrajectory()
    
    # Set the joint names to the gripper joint names
    t.joint_names = arm_commander.robot.get_joint_names(arm_commander.gripper_mvgroup.get_name())
    
    # Initialize a joint trajectory point to represent the goal
    tp = JointTrajectoryPoint()
    
    # Assign the trajectory joint positions to the input positions
    tp.positions = joint_positions
    
    # Set the gripper effort
    tp.effort = [1.0]
    
    tp.time_from_start = rospy.Duration(1.0)
    
    # Append the goal point to the trajectory points
    t.points.append(tp)
    
    # Return the joint trajectory
    return t

# Generate a gripper translation in the direction given by vector
def make_gripper_translation(arm_commander :ArmCommander, min_dist, desired, vector):
    # Initialize the gripper translation object
    g = GripperTranslation()
    
    # Set the direction vector components to the input
    g.direction.vector.x = vector[0]
    g.direction.vector.y = vector[1]
    g.direction.vector.z = vector[2]
    
    # The vector is relative to the gripper frame
    g.direction.header.frame_id = arm_commander.ARM_REF_FRAME
    
    # Assign the min and desired distances from the input
    g.min_distance = min_dist
    g.desired_distance = desired
    
    return g

# Generate a list of possible grasps
def make_grasps(arm_commander, initial_pose_stamped, allowed_touch_objects):
    gripper_open_joints = degrees_to_radians([-20.0, 0.0, -80.0, 0.0, -80.0])
    gripper_close_joints = degrees_to_radians([-15.0, 0.0, -2.0, 0.0, -2.0])
    arm_pos_joints = degrees_to_radians([-4.0, -29.0, -68.0, -180.0, 83.0])

    # Initialize the grasp object
    g = Grasp()
    
    # Set the pre-grasp and grasp postures appropriately
    g.pre_grasp_posture = make_gripper_posture(arm_commander, gripper_open_joints)
    g.grasp_posture = make_gripper_posture(arm_commander, gripper_close_joints)

    # Set the approach and retreat parameters as desired
    g.pre_grasp_approach = make_gripper_translation(arm_commander, 0.001, 0.001, [0, 0, -1])
    g.post_grasp_retreat = make_gripper_translation(arm_commander, 0.1, 0.15, [0, 0, 1])
    
    # Set the first grasp pose to the input pose
    g.grasp_pose = initial_pose_stamped

    # A list to hold the grasps
    grasps = []
    grasps.append(deepcopy(g))

    ideal_roll = 0
    ideal_pitch = 0
    ideal_yaw = 0
    
    step_size = 0.1
    idx = 0.1
    idx_roll = ideal_roll + idx
    idx_pitch = ideal_pitch + idx
    idx_yaw = ideal_yaw + idx
    roll_vals = []
    pitch_vals = []
    yaw_vals = []
    while idx >= -0.1:
        roll_vals.append(idx_roll)
        pitch_vals.append(idx_pitch)
        yaw_vals.append(idx_yaw)
        idx -= step_size
        idx_roll -= step_size
        idx_pitch -= step_size
        idx_yaw -= step_size
        
        # Generate a grasp for each roll pitch and yaw angle
        for r in roll_vals:
            for y in yaw_vals:
                for p in pitch_vals:
                    # Create a quaternion from the Euler angles
                    q = quaternion_from_euler(r, p, y)
                    
                    # Set the grasp pose orientation accordingly
                    g.grasp_pose.pose.orientation.x = q[0]
                    g.grasp_pose.pose.orientation.y = q[1]
                    g.grasp_pose.pose.orientation.z = q[2]
                    g.grasp_pose.pose.orientation.w = q[3]
                    
                    # Set and id for this grasp (simply needs to be unique)
                    g.id = str(len(grasps))
                    
                    # Set the allowed touch objects to the input list
                    g.allowed_touch_objects = allowed_touch_objects
                    
                    # Don't restrict contact force
                    g.max_contact_force = 0
                    
                    # Degrade grasp quality for increasing pitch angles
                    g.grasp_quality = 1.0 - abs(p)
                    
                    # Append the grasp to the list
                    grasps.append(deepcopy(g))
                    print(g)
                    
        print("Generated " + str(len(grasps)) + " poses")
        # Return the list
        return grasps

def grasp_test(arm_commander: ArmCommander,max_attempts=25):
    #clean scene
    arm_commander.scene.remove_world_object("part")
    arm_commander.scene.remove_world_object("table")

    #place scene objects for test
    obj_pos = PoseStamped()
    obj_pos.header.frame_id = arm_commander.ARM_REF_FRAME #todo: test/change reference frame for these objects

    obj_pos.pose.orientation.w = 1.0 #leave other quat. values at default

    obj_pos.pose.position.x = 0.0
    obj_pos.pose.position.y = 0.46
    obj_pos.pose.position.z = 0.025

    table_size = (0.5,0.5,0.05)
    arm_commander.scene.add_box("table", obj_pos, table_size)
    arm_commander.arm_mvgroup.set_support_surface_name("table") #for place operation

    obj_pos.pose.position.x = 0.01
    obj_pos.pose.position.y = 0.29
    obj_pos.pose.position.z = 0.1
    arm_commander.scene.add_cylinder("part", obj_pos, 0.2, 0.04)


    #generate a list of grasp approaches from current eef pose
    #initial pose target
    init_pose = PoseStamped()
    init_pose = arm_commander.get_end_effector_pose()
    grasps = make_grasps(arm_commander, init_pose, ['part'])

    #attempt pick operation and report success/failure
    result = None
    n_attempts = 0
    while result != MoveItErrorCodes.SUCCESS and n_attempts < max_attempts:
        n_attempts += 1
        rospy.loginfo("Pick attempt: " +  str(n_attempts))
        result = arm_commander.arm_mvgroup.pick('part',grasps)
        rospy.sleep(0.2)
