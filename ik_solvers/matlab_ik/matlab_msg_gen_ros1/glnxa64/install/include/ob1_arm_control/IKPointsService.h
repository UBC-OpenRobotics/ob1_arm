// Generated by gencpp from file ob1_arm_control/IKPointsService.msg
// DO NOT EDIT!


#ifndef OB1_ARM_CONTROL_MESSAGE_IKPOINTSSERVICE_H
#define OB1_ARM_CONTROL_MESSAGE_IKPOINTSSERVICE_H

#include <ros/service_traits.h>


#include <ob1_arm_control/IKPointsServiceRequest.h>
#include <ob1_arm_control/IKPointsServiceResponse.h>


namespace ob1_arm_control
{

struct IKPointsService
{

typedef IKPointsServiceRequest Request;
typedef IKPointsServiceResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct IKPointsService
} // namespace ob1_arm_control


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::ob1_arm_control::IKPointsService > {
  static const char* value()
  {
    return "c9a39e8b1d27189ece75a6ccd1218770";
  }

  static const char* value(const ::ob1_arm_control::IKPointsService&) { return value(); }
};

template<>
struct DataType< ::ob1_arm_control::IKPointsService > {
  static const char* value()
  {
    return "ob1_arm_control/IKPointsService";
  }

  static const char* value(const ::ob1_arm_control::IKPointsService&) { return value(); }
};


// service_traits::MD5Sum< ::ob1_arm_control::IKPointsServiceRequest> should match
// service_traits::MD5Sum< ::ob1_arm_control::IKPointsService >
template<>
struct MD5Sum< ::ob1_arm_control::IKPointsServiceRequest>
{
  static const char* value()
  {
    return MD5Sum< ::ob1_arm_control::IKPointsService >::value();
  }
  static const char* value(const ::ob1_arm_control::IKPointsServiceRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::ob1_arm_control::IKPointsServiceRequest> should match
// service_traits::DataType< ::ob1_arm_control::IKPointsService >
template<>
struct DataType< ::ob1_arm_control::IKPointsServiceRequest>
{
  static const char* value()
  {
    return DataType< ::ob1_arm_control::IKPointsService >::value();
  }
  static const char* value(const ::ob1_arm_control::IKPointsServiceRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::ob1_arm_control::IKPointsServiceResponse> should match
// service_traits::MD5Sum< ::ob1_arm_control::IKPointsService >
template<>
struct MD5Sum< ::ob1_arm_control::IKPointsServiceResponse>
{
  static const char* value()
  {
    return MD5Sum< ::ob1_arm_control::IKPointsService >::value();
  }
  static const char* value(const ::ob1_arm_control::IKPointsServiceResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::ob1_arm_control::IKPointsServiceResponse> should match
// service_traits::DataType< ::ob1_arm_control::IKPointsService >
template<>
struct DataType< ::ob1_arm_control::IKPointsServiceResponse>
{
  static const char* value()
  {
    return DataType< ::ob1_arm_control::IKPointsService >::value();
  }
  static const char* value(const ::ob1_arm_control::IKPointsServiceResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // OB1_ARM_CONTROL_MESSAGE_IKPOINTSSERVICE_H