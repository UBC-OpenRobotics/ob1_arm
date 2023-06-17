from geometry_msgs.msg import PoseStamped, Pose, Point, PointStamped, Quaternion
import geometry_msgs.msg as gm
from moveit_commander import MoveGroupCommander,RobotCommander, PlanningSceneInterface
from moveit_msgs.msg import ExecuteTrajectoryAction, ExecuteTrajectoryGoal
import rospy

rospy.init_node('test_spy')

scene = PlanningSceneInterface()

box_pose = PoseStamped()
box_pose.header.frame_id = "interbotix_arm"
box_pose.pose.position.x = 0.3
box_pose.pose.position.y = 0.0
box_pose.pose.position.z = 0.1
box_name = "box"
scene.add_box(box_name, box_pose, size=(0.01, 0.01, 0.01))