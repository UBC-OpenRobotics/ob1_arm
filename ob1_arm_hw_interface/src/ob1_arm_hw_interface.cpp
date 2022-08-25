#include <ob1_arm_hw_interface/ob1_arm_hw_interface.h>

namespace ob1_arm_hw
{
    Ob1ArmHWInterface::Ob1ArmHWInterface(ros::NodeHandle& nh, urdf::Model* urdf_model)
    : ros_control_boilerplate::GenericHWInterface(nh, urdf_model), name_("ob1_arm_hw_interface")
    {
        msgCount = 0;
        bufferHealth = 10;

        // Load rosparams
        ros::NodeHandle rpnh(nh_, "hardware_interface");

        arm_state_sub = nh.subscribe("/arduino/armState", 1, &Ob1ArmHWInterface::armStateCallback, this);
        arm_cmd_pub = nh.advertise<ob1_arm_hw_interface::armCmd>("/arduino/armCmd", 3);
    }

    void Ob1ArmHWInterface::armStateCallback(const ob1_arm_hw_interface::armState::ConstPtr &msg)
    {
        for(int i=0; i<num_joints_; i++){
            joint_velocity_[i] = msg->vel[i]*DEG_TO_RAD; // declared in GenericHWInterface
            joint_position_[i] = msg->angle[i]*DEG_TO_RAD;
        }

        bufferHealth = msgCount - msg->msg_rcv_ctr;
    }

    void Ob1ArmHWInterface::init()
    {
        // Call parent class version of this function
        GenericHWInterface::init();

        ROS_INFO_NAMED(name_, "Ob1ArmHWInterface Ready.");
    }

    void Ob1ArmHWInterface::read(ros::Duration& elapsed_time)
    {
        //do nothing since call back function for subscriber handles reading
    }

    void Ob1ArmHWInterface::write(ros::Duration& elapsed_time)
    {

        bool changeFlag = false;

        for(int i =0; i < num_joints_; i++)
        {
            if(joint_pos_cmd_prev_[i] != joint_position_command_[i])
            {
                changeFlag = true;
                break;
            }
        }

        static ob1_arm_hw_interface::armCmd cmd;

        if(changeFlag)
        {
            for(int i=0; i<num_joints_; i++)
            {
                cmd.angle[i]=joint_position_command_[i]*RAD_TO_DEG;
                cmd.vel[i] = 0;

                joint_pos_cmd_prev_[i] = joint_position_command_[i];
            }

            msgCount++;
            cmd.msg_send_ctr = msgCount;
            cmd.num_joints =  num_joints_;
            arm_cmd_pub.publish(cmd);
        }
    }

    void Ob1ArmHWInterface::enforceLimits(ros::Duration& period)
    {
        // Enforces position and velocity
        pos_jnt_sat_interface_.enforceLimits(period);
    }

}  
