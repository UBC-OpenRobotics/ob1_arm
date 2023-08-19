#ifndef _ROS_SERVICE_MatlabIKService_h
#define _ROS_SERVICE_MatlabIKService_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace ob1_arm_control
{

static const char MATLABIKSERVICE[] = "ob1_arm_control/MatlabIKService";

  class MatlabIKServiceRequest : public ros::Msg
  {
    public:
      uint32_t pose_target_length;
      typedef double _pose_target_type;
      _pose_target_type st_pose_target;
      _pose_target_type * pose_target;
      uint32_t tolerance_length;
      typedef double _tolerance_type;
      _tolerance_type st_tolerance;
      _tolerance_type * tolerance;

    MatlabIKServiceRequest():
      pose_target_length(0), st_pose_target(), pose_target(nullptr),
      tolerance_length(0), st_tolerance(), tolerance(nullptr)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->pose_target_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->pose_target_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->pose_target_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->pose_target_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->pose_target_length);
      for( uint32_t i = 0; i < pose_target_length; i++){
      union {
        double real;
        uint64_t base;
      } u_pose_targeti;
      u_pose_targeti.real = this->pose_target[i];
      *(outbuffer + offset + 0) = (u_pose_targeti.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_pose_targeti.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_pose_targeti.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_pose_targeti.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_pose_targeti.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_pose_targeti.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_pose_targeti.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_pose_targeti.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->pose_target[i]);
      }
      *(outbuffer + offset + 0) = (this->tolerance_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->tolerance_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->tolerance_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->tolerance_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->tolerance_length);
      for( uint32_t i = 0; i < tolerance_length; i++){
      union {
        double real;
        uint64_t base;
      } u_tolerancei;
      u_tolerancei.real = this->tolerance[i];
      *(outbuffer + offset + 0) = (u_tolerancei.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_tolerancei.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_tolerancei.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_tolerancei.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_tolerancei.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_tolerancei.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_tolerancei.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_tolerancei.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->tolerance[i]);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      uint32_t pose_target_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      pose_target_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      pose_target_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      pose_target_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->pose_target_length);
      if(pose_target_lengthT > pose_target_length)
        this->pose_target = (double*)realloc(this->pose_target, pose_target_lengthT * sizeof(double));
      pose_target_length = pose_target_lengthT;
      for( uint32_t i = 0; i < pose_target_length; i++){
      union {
        double real;
        uint64_t base;
      } u_st_pose_target;
      u_st_pose_target.base = 0;
      u_st_pose_target.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_pose_target.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_pose_target.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_pose_target.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_st_pose_target.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_st_pose_target.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_st_pose_target.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_st_pose_target.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->st_pose_target = u_st_pose_target.real;
      offset += sizeof(this->st_pose_target);
        memcpy( &(this->pose_target[i]), &(this->st_pose_target), sizeof(double));
      }
      uint32_t tolerance_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      tolerance_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      tolerance_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      tolerance_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->tolerance_length);
      if(tolerance_lengthT > tolerance_length)
        this->tolerance = (double*)realloc(this->tolerance, tolerance_lengthT * sizeof(double));
      tolerance_length = tolerance_lengthT;
      for( uint32_t i = 0; i < tolerance_length; i++){
      union {
        double real;
        uint64_t base;
      } u_st_tolerance;
      u_st_tolerance.base = 0;
      u_st_tolerance.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_tolerance.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_tolerance.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_tolerance.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_st_tolerance.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_st_tolerance.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_st_tolerance.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_st_tolerance.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->st_tolerance = u_st_tolerance.real;
      offset += sizeof(this->st_tolerance);
        memcpy( &(this->tolerance[i]), &(this->st_tolerance), sizeof(double));
      }
     return offset;
    }

    virtual const char * getType() override { return MATLABIKSERVICE; };
    virtual const char * getMD5() override { return "ad09fc55b0d494343b290d861e0efa46"; };

  };

  class MatlabIKServiceResponse : public ros::Msg
  {
    public:
      typedef const char* _result_type;
      _result_type result;
      typedef double _error_type;
      _error_type error;
      uint32_t joints_length;
      typedef double _joints_type;
      _joints_type st_joints;
      _joints_type * joints;

    MatlabIKServiceResponse():
      result(""),
      error(0),
      joints_length(0), st_joints(), joints(nullptr)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      uint32_t length_result = strlen(this->result);
      varToArr(outbuffer + offset, length_result);
      offset += 4;
      memcpy(outbuffer + offset, this->result, length_result);
      offset += length_result;
      union {
        double real;
        uint64_t base;
      } u_error;
      u_error.real = this->error;
      *(outbuffer + offset + 0) = (u_error.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_error.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_error.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_error.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_error.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_error.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_error.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_error.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->error);
      *(outbuffer + offset + 0) = (this->joints_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->joints_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->joints_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->joints_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->joints_length);
      for( uint32_t i = 0; i < joints_length; i++){
      union {
        double real;
        uint64_t base;
      } u_jointsi;
      u_jointsi.real = this->joints[i];
      *(outbuffer + offset + 0) = (u_jointsi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_jointsi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_jointsi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_jointsi.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_jointsi.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_jointsi.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_jointsi.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_jointsi.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->joints[i]);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      uint32_t length_result;
      arrToVar(length_result, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_result; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_result-1]=0;
      this->result = (char *)(inbuffer + offset-1);
      offset += length_result;
      union {
        double real;
        uint64_t base;
      } u_error;
      u_error.base = 0;
      u_error.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_error.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_error.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_error.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_error.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_error.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_error.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_error.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->error = u_error.real;
      offset += sizeof(this->error);
      uint32_t joints_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      joints_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      joints_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      joints_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->joints_length);
      if(joints_lengthT > joints_length)
        this->joints = (double*)realloc(this->joints, joints_lengthT * sizeof(double));
      joints_length = joints_lengthT;
      for( uint32_t i = 0; i < joints_length; i++){
      union {
        double real;
        uint64_t base;
      } u_st_joints;
      u_st_joints.base = 0;
      u_st_joints.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_joints.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_joints.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_joints.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_st_joints.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_st_joints.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_st_joints.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_st_joints.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->st_joints = u_st_joints.real;
      offset += sizeof(this->st_joints);
        memcpy( &(this->joints[i]), &(this->st_joints), sizeof(double));
      }
     return offset;
    }

    virtual const char * getType() override { return MATLABIKSERVICE; };
    virtual const char * getMD5() override { return "bec3ec873883402ec62934e02f51a48f"; };

  };

  class MatlabIKService {
    public:
    typedef MatlabIKServiceRequest Request;
    typedef MatlabIKServiceResponse Response;
  };

}
#endif
