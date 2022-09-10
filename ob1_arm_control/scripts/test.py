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