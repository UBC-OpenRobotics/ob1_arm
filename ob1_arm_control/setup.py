from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

setup_args = generate_distutils_setup(
    packages=['ob1_arm_control'],
    package_dir={'':'.'},
    requires=['moveit_commander', 'moveit_msgs', 'geometry_msgs', 'rospy', 'numpy', 'tf']
)

setup(**setup_args)