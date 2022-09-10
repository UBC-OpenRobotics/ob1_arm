# ob1_arm_hw_interface

Author:

Yousif El-Wishahy

Email: yel.wishahy@gmail.com

## Summary

Hardware interface for the ob1 robotic arm. This implements a hw interface for the Ros Control plugin (http://wiki.ros.org/ros_control) using a hand ros_control_boilerplate.  

Ros control contains multiple controller plugins to control a robotic arm, including:
* joint_state_controller
    * "Publishes the state of all resources registered to a hardware_interface::JointStateInterface to a topic of type sensor_msgs/JointState."
    
    * This is how MoevIt will receive the robot state which is just the current joint values of the servos as read by the encoders at each joint on the physical arm.

* There are position_controllers, velocity_controllers, and effort_controllers (usually torque or power) that are responsible for commanding the robot joints either inidivually or multiple joints at once. Testing will start by commanding joint positions.

These are the high level controllers that subsribe and publish to moveit arm topics and handle PID. These have already been implemented as part of the ros control package.

To communicate between the hardware (Ardunio connected to servos and encoders) and the controllers, ros control requires an implementation of the class `hardware_interface::RobotHW` that initializes and registers with all the different controller interfaces and reads and writes to and from the hardware. Additionally, ros_control_boilerplate has a generic class `ros_control_boilerplate::GenericHWInterface` that does a lot of the heavy lifting such initializing and registering the controller interfaces as well as handling storing and updating robot states and robot commands. This class inherits and implements the `hardware_interface::RobotHW` class.

In this package I implement a class `Ob1ArmHWInterface` which inherits `ros_control_boilerplate::GenericHWInterface`. 

## Communication Functions

There are 2 main communication functions to handle related to the serial communications:

* Send position or velocity commands over serial to arduino with `Ob1ArmHWInterface::write(ros::Duration& elapsed_time)`

* Read from arduino and update robot state (an `std::vector<double> joint_position_`) with `Ob1ArmHWInterface::write(ros::Duration& elapsed_time)`

Internally, the arm commands and arm states are vectors with joint, velocity, and effort values for each arm joint. This is the information relayed through the controller interface. 

```
  // States
  std::vector<double> joint_position_;
  std::vector<double> joint_velocity_;
  std::vector<double> joint_effort_;

  // Commands
  std::vector<double> joint_position_command_;
  std::vector<double> joint_velocity_command_;
  std::vector<double> joint_effort_command_;
```

## Serial Communication with rosserial

rosserial will allow for the arduino to act as a fully fledged ros node that we can subcribe and publish to. This plugin implements all the communication protocols over UART.

With this method, we use custom ros messages to communicate between the hwardware interface and arduino as we would with any other ros node. Additional we use subscribe and publish to achieve this.

There are 2 custom ros messages:
* armCmd will be used to publish joint/velocity commands to the arduino node.
* armState will subscribe to the arduino node and receive the robot state

```
#armCmd
float32[6] vel #deg/s
float32[6] angle #deg
int32 msg_send_ctr # count sent msgs to detected missed messages
int32 num_joints
```

```
#armState
float32[6] angle # degrees
float32[6] vel # deg/s
int32 msg_rcv_ctr
```

## Setup and Installation 

This package requires ros_control and rosserial, install for ubtunu noetic with the following commands: 

```
sudo apt-get install ros-noetic-ros-control ros-noetic-ros-controllers

sudo apt-get install ros-noetic-rosserial-arduino ros-noetic-rosserial
```

## Integrating rosserial with Arduino firmware

Tutorial: http://wiki.ros.org/rosserial_arduino/Tutorials/Arduino%20IDE%20Setup 

The usage of rosserial is fairly simple. First you need to inlude the ros_lib library (ros.h header) and any custom message headers you will be using.

Assuming the rosserial package is installed on your Ubuntu, you can compile all the required libraries with: 
```
rosrun rosserial_arduino make_libraries.py <arduino libraries directory path> <custom_msg_pkg>
```

In my case, my arduino libraries path available to my arduino IDE is `~/Arduino/libraries/`

Once you have the required libraries, setting up the ros node on your arduino is very simple.

Basic hello world tutorial: http://wiki.ros.org/rosserial_arduino/Tutorials/Hello%20World

## Configuring rosserial arduino parameters 

The hello world tutorial shows a basic example of what you need to do to get a ros node running on arduino. There isn't much more than that on the arduino side. 

On the roscore server, a rosserial python server needs to be setup with certain parameters. This is done in the `ob1_arm_HW.launch` launch file. 

In this file you need to set the correct serial port parameters for the arduino. In my case the serial port is `/dev/ttyACM0` but this will differ.

```
 <!-- ROS SERIAL SERVER (for arduino publishing/subscribing) -->
   <node name="serial_node"        pkg="rosserial_python"      type="serial_node.py">
      <param name="port"              type="string"               value="/dev/ttyACM0"/>
      <param name="baud"              type="int"                  value="57600"/>
   </node>
```

## Launching moveit and hardware interface

Build catkin ws from director 1 level before root directory of repo and source devel/setup.bash.

Run ```roslaunch ob1_arm_hw_interface ob1_arm_HW.launch```

This will all the required nodes for the arm. 



