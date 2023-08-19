#ifndef _ROS_SERVICE_IKPointsService_h
#define _ROS_SERVICE_IKPointsService_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "ob1_arm_control/JointTarget.h"
#include "geometry_msgs/Pose.h"

namespace ob1_arm_control
{

static const char IKPOINTSSERVICE[] = "ob1_arm_control/IKPointsService";

  class IKPointsServiceRequest : public ros::Msg
  {
    public:
      typedef const char* _request_type;
      _request_type request;
      typedef geometry_msgs::Pose _pose_type;
      _pose_type pose;
      typedef uint32_t _num_pts_type;
      _num_pts_type num_pts;
      typedef float _tolerance_type;
      _tolerance_type tolerance;
      typedef float _distance_type;
      _distance_type distance;

    IKPointsServiceRequest():
      request(""),
      pose(),
      num_pts(0),
      tolerance(0),
      distance(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      uint32_t length_request = strlen(this->request);
      varToArr(outbuffer + offset, length_request);
      offset += 4;
      memcpy(outbuffer + offset, this->request, length_request);
      offset += length_request;
      offset += this->pose.serialize(outbuffer + offset);
      *(outbuffer + offset + 0) = (this->num_pts >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->num_pts >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->num_pts >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->num_pts >> (8 * 3)) & 0xFF;
      offset += sizeof(this->num_pts);
      union {
        float real;
        uint32_t base;
      } u_tolerance;
      u_tolerance.real = this->tolerance;
      *(outbuffer + offset + 0) = (u_tolerance.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_tolerance.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_tolerance.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_tolerance.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->tolerance);
      union {
        float real;
        uint32_t base;
      } u_distance;
      u_distance.real = this->distance;
      *(outbuffer + offset + 0) = (u_distance.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_distance.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_distance.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_distance.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->distance);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      uint32_t length_request;
      arrToVar(length_request, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_request; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_request-1]=0;
      this->request = (char *)(inbuffer + offset-1);
      offset += length_request;
      offset += this->pose.deserialize(inbuffer + offset);
      this->num_pts =  ((uint32_t) (*(inbuffer + offset)));
      this->num_pts |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->num_pts |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->num_pts |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->num_pts);
      union {
        float real;
        uint32_t base;
      } u_tolerance;
      u_tolerance.base = 0;
      u_tolerance.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_tolerance.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_tolerance.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_tolerance.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->tolerance = u_tolerance.real;
      offset += sizeof(this->tolerance);
      union {
        float real;
        uint32_t base;
      } u_distance;
      u_distance.base = 0;
      u_distance.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_distance.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_distance.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_distance.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->distance = u_distance.real;
      offset += sizeof(this->distance);
     return offset;
    }

    virtual const char * getType() override { return IKPOINTSSERVICE; };
    virtual const char * getMD5() override { return "277fc6b60045d7610e020622b515d0d7"; };

  };

  class IKPointsServiceResponse : public ros::Msg
  {
    public:
      uint32_t pose_targets_length;
      typedef geometry_msgs::Pose _pose_targets_type;
      _pose_targets_type st_pose_targets;
      _pose_targets_type * pose_targets;
      uint32_t joint_targets_length;
      typedef ob1_arm_control::JointTarget _joint_targets_type;
      _joint_targets_type st_joint_targets;
      _joint_targets_type * joint_targets;
      typedef bool _condition_type;
      _condition_type condition;

    IKPointsServiceResponse():
      pose_targets_length(0), st_pose_targets(), pose_targets(nullptr),
      joint_targets_length(0), st_joint_targets(), joint_targets(nullptr),
      condition(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->pose_targets_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->pose_targets_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->pose_targets_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->pose_targets_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->pose_targets_length);
      for( uint32_t i = 0; i < pose_targets_length; i++){
      offset += this->pose_targets[i].serialize(outbuffer + offset);
      }
      *(outbuffer + offset + 0) = (this->joint_targets_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->joint_targets_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->joint_targets_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->joint_targets_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->joint_targets_length);
      for( uint32_t i = 0; i < joint_targets_length; i++){
      offset += this->joint_targets[i].serialize(outbuffer + offset);
      }
      union {
        bool real;
        uint8_t base;
      } u_condition;
      u_condition.real = this->condition;
      *(outbuffer + offset + 0) = (u_condition.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->condition);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      uint32_t pose_targets_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      pose_targets_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      pose_targets_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      pose_targets_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->pose_targets_length);
      if(pose_targets_lengthT > pose_targets_length)
        this->pose_targets = (geometry_msgs::Pose*)realloc(this->pose_targets, pose_targets_lengthT * sizeof(geometry_msgs::Pose));
      pose_targets_length = pose_targets_lengthT;
      for( uint32_t i = 0; i < pose_targets_length; i++){
      offset += this->st_pose_targets.deserialize(inbuffer + offset);
        memcpy( &(this->pose_targets[i]), &(this->st_pose_targets), sizeof(geometry_msgs::Pose));
      }
      uint32_t joint_targets_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      joint_targets_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      joint_targets_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      joint_targets_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->joint_targets_length);
      if(joint_targets_lengthT > joint_targets_length)
        this->joint_targets = (ob1_arm_control::JointTarget*)realloc(this->joint_targets, joint_targets_lengthT * sizeof(ob1_arm_control::JointTarget));
      joint_targets_length = joint_targets_lengthT;
      for( uint32_t i = 0; i < joint_targets_length; i++){
      offset += this->st_joint_targets.deserialize(inbuffer + offset);
        memcpy( &(this->joint_targets[i]), &(this->st_joint_targets), sizeof(ob1_arm_control::JointTarget));
      }
      union {
        bool real;
        uint8_t base;
      } u_condition;
      u_condition.base = 0;
      u_condition.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->condition = u_condition.real;
      offset += sizeof(this->condition);
     return offset;
    }

    virtual const char * getType() override { return IKPOINTSSERVICE; };
    virtual const char * getMD5() override { return "d33dd84b62a8467976be018c77f8d97c"; };

  };

  class IKPointsService {
    public:
    typedef IKPointsServiceRequest Request;
    typedef IKPointsServiceResponse Response;
  };

}
#endif
