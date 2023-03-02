// Generated by gencpp from file relaxed_ik/RelaxedIKService.msg
// DO NOT EDIT!


#ifndef RELAXED_IK_MESSAGE_RELAXEDIKSERVICE_H
#define RELAXED_IK_MESSAGE_RELAXEDIKSERVICE_H

#include <ros/service_traits.h>


#include <relaxed_ik/RelaxedIKServiceRequest.h>
#include <relaxed_ik/RelaxedIKServiceResponse.h>


namespace relaxed_ik
{

struct RelaxedIKService
{

typedef RelaxedIKServiceRequest Request;
typedef RelaxedIKServiceResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct RelaxedIKService
} // namespace relaxed_ik


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::relaxed_ik::RelaxedIKService > {
  static const char* value()
  {
    return "ca8e7e130f99c7d2092a0eb23c5fc3a9";
  }

  static const char* value(const ::relaxed_ik::RelaxedIKService&) { return value(); }
};

template<>
struct DataType< ::relaxed_ik::RelaxedIKService > {
  static const char* value()
  {
    return "relaxed_ik/RelaxedIKService";
  }

  static const char* value(const ::relaxed_ik::RelaxedIKService&) { return value(); }
};


// service_traits::MD5Sum< ::relaxed_ik::RelaxedIKServiceRequest> should match
// service_traits::MD5Sum< ::relaxed_ik::RelaxedIKService >
template<>
struct MD5Sum< ::relaxed_ik::RelaxedIKServiceRequest>
{
  static const char* value()
  {
    return MD5Sum< ::relaxed_ik::RelaxedIKService >::value();
  }
  static const char* value(const ::relaxed_ik::RelaxedIKServiceRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::relaxed_ik::RelaxedIKServiceRequest> should match
// service_traits::DataType< ::relaxed_ik::RelaxedIKService >
template<>
struct DataType< ::relaxed_ik::RelaxedIKServiceRequest>
{
  static const char* value()
  {
    return DataType< ::relaxed_ik::RelaxedIKService >::value();
  }
  static const char* value(const ::relaxed_ik::RelaxedIKServiceRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::relaxed_ik::RelaxedIKServiceResponse> should match
// service_traits::MD5Sum< ::relaxed_ik::RelaxedIKService >
template<>
struct MD5Sum< ::relaxed_ik::RelaxedIKServiceResponse>
{
  static const char* value()
  {
    return MD5Sum< ::relaxed_ik::RelaxedIKService >::value();
  }
  static const char* value(const ::relaxed_ik::RelaxedIKServiceResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::relaxed_ik::RelaxedIKServiceResponse> should match
// service_traits::DataType< ::relaxed_ik::RelaxedIKService >
template<>
struct DataType< ::relaxed_ik::RelaxedIKServiceResponse>
{
  static const char* value()
  {
    return DataType< ::relaxed_ik::RelaxedIKService >::value();
  }
  static const char* value(const ::relaxed_ik::RelaxedIKServiceResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // RELAXED_IK_MESSAGE_RELAXEDIKSERVICE_H