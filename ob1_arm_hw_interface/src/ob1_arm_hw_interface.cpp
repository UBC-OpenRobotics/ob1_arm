#include <ob1_arm_hw_interface/ob1_arm_hw_interface.h>

// ROS parameter loading
#include <rosparam_shortcuts/rosparam_shortcuts.h>

namespace ob1_arm_hw
{
Ob1ArmHWInterface::Ob1ArmHWInterface(ros::NodeHandle& nh, urdf::Model* urdf_model)
  : ros_control_boilerplate::GenericHWInterface(nh, urdf_model), name_("ob1_arm_hw_interface")
{
  // Load rosparams
  ros::NodeHandle rpnh(nh_, "hardware_interface");
//   std::size_t error = 0;
//   error += !rosparam_shortcuts::get(name_, rpnh, "sim_control_mode", sim_control_mode_);
//   if (error)
//   {
//     ROS_WARN_STREAM_NAMED(name_, "SimHWInterface now requires the following config in the yaml:");
//     ROS_WARN_STREAM_NAMED(name_, "   sim_control_mode: 0 # 0: position, 1: velocity");
//   }
//   rosparam_shortcuts::shutdownIfError(name_, error);
}

void Ob1ArmHWInterface::init()
{
  // Call parent class version of this function
  GenericHWInterface::init();

  ROS_INFO_NAMED(name_, "Ob1ArmHWInterface Ready.");
}

void Ob1ArmHWInterface::read(ros::Duration& elapsed_time)
{
  // No need to read since our write() command populates our state for us
}

void Ob1ArmHWInterface::write(ros::Duration& elapsed_time)
{

}

void Ob1ArmHWInterface::enforceLimits(ros::Duration& period)
{
  // Enforces position and velocity
  pos_jnt_sat_interface_.enforceLimits(period);
}

}  // namespace ros_control_boilerplate
