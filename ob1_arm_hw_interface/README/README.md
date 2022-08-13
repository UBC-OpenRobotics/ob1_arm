# ob1_arm_hw_interface

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

## Serial Communication

Read/write will happen between arduino and ros machine (linux box likely) over serial ports. JSON data will be serialized in the write() cmd, sent as a stream to arduino, and deserialized. The ArduinoJson library (https://arduinojson.org/) will facilitate this.

### Output (Joint commands)

Example arm command message that will be sent to arduino (this is a 'prettified' version, but unprettified version will be sent to save bytes):

```
{
  "msg_counter": 27,
  "num_joints": 6,
  "time": 1351824120,
  "angle": [
    48.75608,
    2.302038,
    2.302038,
    2.302038,
    2.302038,
    2.302038
  ],
  "vel": [
    48.75608,
    2.302038,
    2.302038,
    2.302038,
    2.302038,
    2.302038
  ]
}
```

### Input (Telemetry)

```
{
  "msg_rcv_counter": 25,
  "read_timestamp": 1351824120,
  "sync_time": 234264,
  "angle": [
    48.75608,
    2.302038,
    2.302038,
    2.302038,
    2.302038,
    2.302038
  ],
  "vel": [
    48.75608,
    2.302038,
    2.302038,
    2.302038,
    2.302038,
    2.302038
  ]
}



