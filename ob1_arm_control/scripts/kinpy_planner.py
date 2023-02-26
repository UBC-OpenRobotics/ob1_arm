#! /usr/bin/env python
import kinpy as kp
import rospkg
import numpy as np
import rospy

rp = rospkg.RosPack()
urdf_path = rp.get_path('ob1_arm_description') + "/urdf/main.urdf"

arm = kp.build_serial_chain_from_urdf(
    open(urdf_path).read(),
    root_link_name="ob1_arm_base_link",
    end_link_name="ob1_arm_eef_link"
)
fk_solution = arm.forward_kinematics([0,0,0,0,0])
sol = {'ob1_arm_gripper_base_link':fk_solution}
tf = kp.Transform(np.array([0,0,0,1]),np.array([0.1,0.1,0.2]))
ik_sol = arm.inverse_kinematics(tf)
print(fk_solution)
print(ik_sol)

viz = kp.Visualizer()
viz.add_robot(sol, arm.visuals_map(), axes=True)
viz.spin()
