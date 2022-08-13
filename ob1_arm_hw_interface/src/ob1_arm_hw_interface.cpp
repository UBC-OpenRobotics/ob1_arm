#include <ob1_arm_hw_interface/ob1_arm_hw_interface.h>

// ROS parameter loading
#include <rosparam_shortcuts/rosparam_shortcuts.h>
#include <arduino_json/ArduinoJson-v6.19.4.h>

namespace ob1_arm_hw
{
    Ob1ArmHWInterface::Ob1ArmHWInterface(ros::NodeHandle& nh, urdf::Model* urdf_model)
    : ros_control_boilerplate::GenericHWInterface(nh, urdf_model), name_("ob1_arm_hw_interface")
    {
        // Load rosparams
        ros::NodeHandle rpnh(nh_, "hardware_interface");
    }

    void Ob1ArmHWInterface::init()
    {
        // Call parent class version of this function
        GenericHWInterface::init();
        msgCount_ = 0;

        //need 2 serial ports for comms
        arduinoOutput.open("/dev/ttyACM0"); //output serial port to write to
        arduinoInput.open("/dev/ttyACM0"); //input serial port to read from
        ROS_INFO_NAMED(name_, "Ob1ArmHWInterface: Opened serial port streams");

        ROS_INFO_NAMED(name_, "Ob1ArmHWInterface Ready.");
    }

    void Ob1ArmHWInterface::close_serial()
    {
        arduinoOutput.close();
        arduinoInput.close();
        ROS_INFO_NAMED(name_, "Ob1ArmHWInterface: Closed serial port streams");
    }

    void Ob1ArmHWInterface::read(ros::Duration& elapsed_time)
    {
        /** Need to populate the following data for robot state
         *   // States
            std::vector<double> joint_position_;
            std::vector<double> joint_velocity_;
        */
       StaticJsonDocument<256> jsonMsg;
       DeserializationError err = deserializeJson(jsonMsg, arduinoInput);

       if(err == DeserializationError::Ok)
       {
            for(int i = 0; i < num_joints_; i++){
                joint_position_[i] = jsonMsg["angle"][i];
                joint_velocity_[i] = jsonMsg["vel"][i];
            }
       }

    }

    void Ob1ArmHWInterface::write(ros::Duration& elapsed_time)
    {
        /**
         angle, vel and effort commands for joints 
         are stored and updated in these variables internally.

         std::vector<double> joint_position_command_;
         std::vector<double> joint_velocity_command_;
        */

        //see readme for example message
        //get json file size with tool: https://arduinojson.org/v6/assistant/

        StaticJsonDocument<256> jsonMsg;

        //populate json msg
        //TODO: radians -> degree conversion if neccessary
        // does robot accept radians or degrees? 
        jsonMsg["msg_counter"] = msgCount_;
        jsonMsg["num_joints"] = num_joints_;
        for(int i = 0; i < num_joints_; i++){
            jsonMsg["angle"][i] = joint_position_command_[i];
            jsonMsg["vel"][i] = joint_velocity_command_[i];
        }

        //write to serial port stream
        serializeJson(jsonMsg, arduinoOutput);
    }

    void Ob1ArmHWInterface::enforceLimits(ros::Duration& period)
    {
        // Enforces position and velocity
        pos_jnt_sat_interface_.enforceLimits(period);
    }

}  // namespace ros_control_boilerplate
