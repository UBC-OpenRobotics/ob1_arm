#import arm commander
from ob1_arm_control.arm_commander import ArmCommander
#import other utilities
from geometry_msgs.msg import PoseStamped
from ob1_arm_control.tf_helpers import broadcast_point, broadcast_pose
import numpy as np

arm_commander = ArmCommander(sample_timeout=25, sample_attempts=10)

#move end effector to x,y,z relative to base link
position = [0.1, 0.3, 0.2] #x,y,z [metres]
result = arm_commander.go_position(position)
if result:
    print('go position succeeded')

#move each arm joint to angle
joints = [np.pi/3] * arm_commander._num_joints #angle pi for each arm joint [radians]
result = arm_commander.go_joint(joints)
if result:
    print('go joint succeeded')

#move end effector to position (point x,y,z) and orientation (quaterionion x,y,z,w)
#must specify reference frame of pose
ps = PoseStamped()
ps.header.frame_id = arm_commander.WORLD_REF_FRAME
ps.pose.position.x = 0.1
ps.pose.position.y = 0.2
ps.pose.position.z = 0.25
ps.pose.orientation.w = 1.0
result = arm_commander.go_pose(ps)
if result:
    print('go pose succeeded')
    
#move end effector pose upwards 0.1m
ps = arm_commander.get_end_effector_pose()
ps.pose.position.z = ps.pose.position.z + 0.1
result = arm_commander.go_pose(ps)
if result:
    print('go pose succeeded')
