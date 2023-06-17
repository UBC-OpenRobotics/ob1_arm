from geometry_msgs.msg import PoseStamped, Pose, Point, PointStamped, Quaternion
import geometry_msgs.msg as gm
from moveit_commander import MoveGroupCommander,RobotCommander, PlanningSceneInterface
from moveit_msgs.msg import ExecuteTrajectoryAction, ExecuteTrajectoryGoal
import rospy
import actionlib
import sys

rospy.init_node('test_spy')

# print(rospy.get_param('/rx150'))

# scene = PlanningSceneInterface()

# box_pose = PoseStamped()
# box_pose.header.frame_id = "interbotix_arm"
# box_pose.pose.position.x = 0.3
# box_pose.pose.position.y = 0.0
# box_pose.pose.position.z = 0.1
# box_name = "box"
# scene.add_box(box_name, box_pose, size=(0.01, 0.01, 0.01))

robotCommander = RobotCommander('rx150/robot_description')
# planningSceneInterface = PlanningSceneInterface()
print(robotCommander.get_group_names())
#init arm move group 
mvgroup:MoveGroupCommander = MoveGroupCommander('interbotix_arm',robot_description='rx150/robot_description',ns='rx150')
grippergroup:MoveGroupCommander = MoveGroupCommander('interbotix_gripper',robot_description='rx150/robot_description',ns='rx150')
# mvgroup = MoveGroupCommander('interbotix_arm',robot_description='rx150/robot_description',ns='rx150')
# grippergroup = MoveGroupCommander('interbotix_gripper',robot_description='rx150/robot_description',ns='rx150')

# mvgroup_client = actionlib.SimpleActionClient('execute_trajectory', ExecuteTrajectoryAction)
# mvgroup_client.wait_for_server()
# rospy.loginfo('Execute Trajectory server is availabe for interbotix_arm')

# grippergroup_client = actionlib.SimpleActionClient('execute_trajectory', ExecuteTrajectoryAction)
# grippergroup_client.wait_for_server()
# rospy.loginfo('Execute Trajectory server is available for interbotix_gripper')

# mvgroup.set_planning_time(60)
# mvgroup.set_num_planning_attempts(10)
# p = Pose()
# p.orientation.w = 0
# p.position.x = 0.30
# p.position.y = 0
# p.position.z = 0.09


# mvgroup.set_pose_target(p)
# mvgroup.go()
# mvgroup.wait_for_result()

# We can get the name of the reference frame for this robot:
planning_frame = mvgroup.get_planning_frame()
print("============ Planning frame: %s" % planning_frame)

# We can also print the name of the end-effector link for this group:
eef_link = mvgroup.get_end_effector_link()
print("============ End effector link: %s" % eef_link)

# Sometimes for debugging it is useful to print the entire state of the
# robot:
print("============ Printing robot state")
print(robotCommander.get_current_state())
print("")

grippergroup.set_planning_time(60)
grippergroup.set_num_planning_attempts(10)

print(grippergroup.get_joints())

joint_goal = grippergroup.get_current_joint_values()
gripper_pose = [0.025, -0.025]

grippergroup.set_joint_value_target(gripper_pose)
grippergroup.go()
grippergroup.stop()


# plan = mvgroup.plan()
# mvgroup_goal = ExecuteTrajectoryGoal()
# mvgroup_goal.trajectory = plan

# mvgroup_client.send_goal(p)
# mvgroup_client.wait_for_result()
