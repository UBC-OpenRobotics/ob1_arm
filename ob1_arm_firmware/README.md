# ob1 arm firmware

Firmware for ob1 arm using arduino framework for now. (Not a ros library).

Libraries used:
- rosserial 
    - Ros Package for linux side: (http://wiki.ros.org/rosserial/)
    - Arduino - https://github.com/frankjoshua/rosserial_arduino_lib

## How to build rosserial client lib for different mcus
todo

## Install rosserial server pkg for ROS

```sudo apt-get install ros-noetic-rosserial*```

## How to build custom ros messages for rosserial

http://wiki.ros.org/rosserial/Tutorials/Adding%20Other%20Messages

```
source /opt/ros/noetic/setup.bash
cd <path to arm_ws>
catkin build
source devel/setup.bash
rosrun rosserial_client make_libraries <path to build destination>
```

This will build a `ros_lib` folder containing c message headers for all ros messages.

The latest message headers for ob1_arm packages are contained in the `ros_lib` folder in this directory.