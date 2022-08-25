#ifndef OB1_ARM_HW_INTERFACE_H
#define OB1_ARM_HW_INTERFACE_H

#include <ros_control_boilerplate/generic_hw_interface.h>
#include <iostream>
#include <fstream>
// ROS parameter loading
#include <rosparam_shortcuts/rosparam_shortcuts.h>
#include <arduino_json/ArduinoJson-v6.19.4.h>
#include "std_msgs/String.h"
#include <sstream>
#include <string>
#include <iostream>

#include <ob1_arm_hw_interface/armCmd.h>
#include <ob1_arm_hw_interface/armState.h>

#define DEG_TO_RAD 0.01745329251
#define RAD_TO_DEG 57.2957795131
#define DESIRED_BUFFERED_POINTS 12

namespace ob1_arm_hw
{
    /** \brief Hardware interface for a robot */
    class Ob1ArmHWInterface : public ros_control_boilerplate::GenericHWInterface
    {
    public:
        /**
         * \brief Constructor
         * \param nh - Node handle for topics.
         */
        Ob1ArmHWInterface(ros::NodeHandle& nh, urdf::Model* urdf_model = NULL);

        /** \brief Initialize the robot hardware interface */
        virtual void init();

        /** \brief Read the state from the robot hardware. */
        virtual void read(ros::Duration& elapsed_time);

        /** \brief Write the command to the robot hardware. */
        virtual void write(ros::Duration& elapsed_time);

        //   /** \breif Enforce limits for all values before writing */
        virtual void enforceLimits(ros::Duration& period);

    protected:
        // Name of this class
        std::string name_;
        long msgCount;
        long bufferHealth;

        //previous joint state
        std::vector<double> joint_pos_cmd_prev_;

        //serial comms
        // std::ofstream arduinoOutput;
        // ob1_arm_hw::ArduinoSerial arduinoInput = ob1_arm_hw::ArduinoSerial();

        //debug topics
        // ros::Publisher writeDebugPub;
        // ros::Publisher readDebugPub;

        //rosserial comms
        ros::Subscriber arm_state_sub;
        void armStateCallback(const ob1_arm_hw_interface::armState::ConstPtr &msg);
        ros::Publisher arm_cmd_pub;

    };  

}  

#endif