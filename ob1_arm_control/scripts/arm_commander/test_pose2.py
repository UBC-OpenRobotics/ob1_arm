from geometry_msgs.msg import PoseStamped, Pose, Point, PointStamped, Quaternion
import geometry_msgs.msg as gm
from moveit_commander import MoveGroupCommander,RobotCommander, PlanningSceneInterface
import rospy

rospy.init_node('test_spy')
# print(rospy.get_param('/rx150'))

# scene = PlanningSceneInterface()

# box_pose = PoseStamped()
# box_pose.header.frame_id = "world"
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

mvgroup.set_planning_time(60)
mvgroup.set_num_planning_attempts(10)

# joint_goal = mvgroup.get_current_joint_values()

#mvgroup.go([-1.3860583782196045, 0.010135923322290182, -0.02006217818260193, 0.7439807057380676, -0.016873789951205254], wait=True)
#mvgroup.stop()

grippergroup.go([0.0, 0.025, -0.025], wait=True)

# [-1.0860583782196045, 0.006135923322290182, 0.30526217818260193, 0.7439807057380676, -0.016873789951205254, -0.7700583934783936, 0.012047873809933662, -0.012047873809933662]