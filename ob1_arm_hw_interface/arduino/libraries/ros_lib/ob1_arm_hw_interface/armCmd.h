#ifndef _ROS_ob1_arm_hw_interface_armCmd_h
#define _ROS_ob1_arm_hw_interface_armCmd_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace ob1_arm_hw_interface
{

  class armCmd : public ros::Msg
  {
    public:
      float vel[6];
      float angle[6];
      typedef int32_t _msg_send_ctr_type;
      _msg_send_ctr_type msg_send_ctr;
      typedef int32_t _num_joints_type;
      _num_joints_type num_joints;

    armCmd():
      vel(),
      angle(),
      msg_send_ctr(0),
      num_joints(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      for( uint32_t i = 0; i < 6; i++){
      union {
        float real;
        uint32_t base;
      } u_veli;
      u_veli.real = this->vel[i];
      *(outbuffer + offset + 0) = (u_veli.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_veli.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_veli.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_veli.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->vel[i]);
      }
      for( uint32_t i = 0; i < 6; i++){
      union {
        float real;
        uint32_t base;
      } u_anglei;
      u_anglei.real = this->angle[i];
      *(outbuffer + offset + 0) = (u_anglei.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_anglei.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_anglei.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_anglei.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->angle[i]);
      }
      union {
        int32_t real;
        uint32_t base;
      } u_msg_send_ctr;
      u_msg_send_ctr.real = this->msg_send_ctr;
      *(outbuffer + offset + 0) = (u_msg_send_ctr.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_msg_send_ctr.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_msg_send_ctr.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_msg_send_ctr.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->msg_send_ctr);
      union {
        int32_t real;
        uint32_t base;
      } u_num_joints;
      u_num_joints.real = this->num_joints;
      *(outbuffer + offset + 0) = (u_num_joints.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_num_joints.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_num_joints.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_num_joints.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->num_joints);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      for( uint32_t i = 0; i < 6; i++){
      union {
        float real;
        uint32_t base;
      } u_veli;
      u_veli.base = 0;
      u_veli.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_veli.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_veli.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_veli.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->vel[i] = u_veli.real;
      offset += sizeof(this->vel[i]);
      }
      for( uint32_t i = 0; i < 6; i++){
      union {
        float real;
        uint32_t base;
      } u_anglei;
      u_anglei.base = 0;
      u_anglei.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_anglei.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_anglei.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_anglei.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->angle[i] = u_anglei.real;
      offset += sizeof(this->angle[i]);
      }
      union {
        int32_t real;
        uint32_t base;
      } u_msg_send_ctr;
      u_msg_send_ctr.base = 0;
      u_msg_send_ctr.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_msg_send_ctr.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_msg_send_ctr.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_msg_send_ctr.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->msg_send_ctr = u_msg_send_ctr.real;
      offset += sizeof(this->msg_send_ctr);
      union {
        int32_t real;
        uint32_t base;
      } u_num_joints;
      u_num_joints.base = 0;
      u_num_joints.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_num_joints.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_num_joints.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_num_joints.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->num_joints = u_num_joints.real;
      offset += sizeof(this->num_joints);
     return offset;
    }

    virtual const char * getType() override { return "ob1_arm_hw_interface/armCmd"; };
    virtual const char * getMD5() override { return "8b54c869722a6f46d221c71294caa2ad"; };

  };

}
#endif
