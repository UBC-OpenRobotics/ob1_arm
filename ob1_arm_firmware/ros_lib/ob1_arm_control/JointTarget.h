#ifndef _ROS_ob1_arm_control_JointTarget_h
#define _ROS_ob1_arm_control_JointTarget_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace ob1_arm_control
{

  class JointTarget : public ros::Msg
  {
    public:
      uint32_t joint_target_length;
      typedef double _joint_target_type;
      _joint_target_type st_joint_target;
      _joint_target_type * joint_target;

    JointTarget():
      joint_target_length(0), st_joint_target(), joint_target(nullptr)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->joint_target_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->joint_target_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->joint_target_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->joint_target_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->joint_target_length);
      for( uint32_t i = 0; i < joint_target_length; i++){
      union {
        double real;
        uint64_t base;
      } u_joint_targeti;
      u_joint_targeti.real = this->joint_target[i];
      *(outbuffer + offset + 0) = (u_joint_targeti.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_joint_targeti.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_joint_targeti.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_joint_targeti.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_joint_targeti.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_joint_targeti.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_joint_targeti.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_joint_targeti.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->joint_target[i]);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      uint32_t joint_target_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      joint_target_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      joint_target_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      joint_target_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->joint_target_length);
      if(joint_target_lengthT > joint_target_length)
        this->joint_target = (double*)realloc(this->joint_target, joint_target_lengthT * sizeof(double));
      joint_target_length = joint_target_lengthT;
      for( uint32_t i = 0; i < joint_target_length; i++){
      union {
        double real;
        uint64_t base;
      } u_st_joint_target;
      u_st_joint_target.base = 0;
      u_st_joint_target.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_joint_target.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_joint_target.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_joint_target.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_st_joint_target.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_st_joint_target.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_st_joint_target.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_st_joint_target.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->st_joint_target = u_st_joint_target.real;
      offset += sizeof(this->st_joint_target);
        memcpy( &(this->joint_target[i]), &(this->st_joint_target), sizeof(double));
      }
     return offset;
    }

    virtual const char * getType() override { return "ob1_arm_control/JointTarget"; };
    virtual const char * getMD5() override { return "133ac1ee056e983bab7d9619a1e96ecf"; };

  };

}
#endif
