#ifndef OB1_ARM_HW_INTERFACE_H
#define OB1_ARM_HW_INTERFACE_H

#include <ros_control_boilerplate/generic_hw_interface.h>
#include <iostream>
#include <fstream>

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

        void close_serial();

        //   /** \breif Enforce limits for all values before writing */
        virtual void enforceLimits(ros::Duration& period);

    protected:
        // Name of this class
        std::string name_;
        long msgCount_;
        std::ofstream arduinoOutput;
        std::ifstream arduinoInput;

    }; 

}  

#endif