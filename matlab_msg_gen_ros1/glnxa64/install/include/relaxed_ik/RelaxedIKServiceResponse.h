// Generated by gencpp from file relaxed_ik/RelaxedIKServiceResponse.msg
// DO NOT EDIT!


#ifndef RELAXED_IK_MESSAGE_RELAXEDIKSERVICERESPONSE_H
#define RELAXED_IK_MESSAGE_RELAXEDIKSERVICERESPONSE_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <relaxed_ik/JointAngles.h>

namespace relaxed_ik
{
template <class ContainerAllocator>
struct RelaxedIKServiceResponse_
{
  typedef RelaxedIKServiceResponse_<ContainerAllocator> Type;

  RelaxedIKServiceResponse_()
    : joint_angles()  {
    }
  RelaxedIKServiceResponse_(const ContainerAllocator& _alloc)
    : joint_angles(_alloc)  {
  (void)_alloc;
    }



   typedef  ::relaxed_ik::JointAngles_<ContainerAllocator>  _joint_angles_type;
  _joint_angles_type joint_angles;





  typedef boost::shared_ptr< ::relaxed_ik::RelaxedIKServiceResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::relaxed_ik::RelaxedIKServiceResponse_<ContainerAllocator> const> ConstPtr;

}; // struct RelaxedIKServiceResponse_

typedef ::relaxed_ik::RelaxedIKServiceResponse_<std::allocator<void> > RelaxedIKServiceResponse;

typedef boost::shared_ptr< ::relaxed_ik::RelaxedIKServiceResponse > RelaxedIKServiceResponsePtr;
typedef boost::shared_ptr< ::relaxed_ik::RelaxedIKServiceResponse const> RelaxedIKServiceResponseConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::relaxed_ik::RelaxedIKServiceResponse_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::relaxed_ik::RelaxedIKServiceResponse_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::relaxed_ik::RelaxedIKServiceResponse_<ContainerAllocator1> & lhs, const ::relaxed_ik::RelaxedIKServiceResponse_<ContainerAllocator2> & rhs)
{
  return lhs.joint_angles == rhs.joint_angles;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::relaxed_ik::RelaxedIKServiceResponse_<ContainerAllocator1> & lhs, const ::relaxed_ik::RelaxedIKServiceResponse_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace relaxed_ik

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::relaxed_ik::RelaxedIKServiceResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::relaxed_ik::RelaxedIKServiceResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::relaxed_ik::RelaxedIKServiceResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::relaxed_ik::RelaxedIKServiceResponse_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::relaxed_ik::RelaxedIKServiceResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::relaxed_ik::RelaxedIKServiceResponse_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::relaxed_ik::RelaxedIKServiceResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "f7b6cdcd8a9f344d4fe1265742fac728";
  }

  static const char* value(const ::relaxed_ik::RelaxedIKServiceResponse_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xf7b6cdcd8a9f344dULL;
  static const uint64_t static_value2 = 0x4fe1265742fac728ULL;
};

template<class ContainerAllocator>
struct DataType< ::relaxed_ik::RelaxedIKServiceResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "relaxed_ik/RelaxedIKServiceResponse";
  }

  static const char* value(const ::relaxed_ik::RelaxedIKServiceResponse_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::relaxed_ik::RelaxedIKServiceResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "JointAngles joint_angles\n"
"\n"
"================================================================================\n"
"MSG: relaxed_ik/JointAngles\n"
"std_msgs/Header header\n"
"std_msgs/Float32[] angles\n"
"\n"
"================================================================================\n"
"MSG: std_msgs/Header\n"
"# Standard metadata for higher-level stamped data types.\n"
"# This is generally used to communicate timestamped data \n"
"# in a particular coordinate frame.\n"
"# \n"
"# sequence ID: consecutively increasing ID \n"
"uint32 seq\n"
"#Two-integer timestamp that is expressed as:\n"
"# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')\n"
"# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')\n"
"# time-handling sugar is provided by the client library\n"
"time stamp\n"
"#Frame this data is associated with\n"
"string frame_id\n"
"\n"
"================================================================================\n"
"MSG: std_msgs/Float32\n"
"float32 data\n"
;
  }

  static const char* value(const ::relaxed_ik::RelaxedIKServiceResponse_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::relaxed_ik::RelaxedIKServiceResponse_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.joint_angles);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct RelaxedIKServiceResponse_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::relaxed_ik::RelaxedIKServiceResponse_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::relaxed_ik::RelaxedIKServiceResponse_<ContainerAllocator>& v)
  {
    s << indent << "joint_angles: ";
    s << std::endl;
    Printer< ::relaxed_ik::JointAngles_<ContainerAllocator> >::stream(s, indent + "  ", v.joint_angles);
  }
};

} // namespace message_operations
} // namespace ros

#endif // RELAXED_IK_MESSAGE_RELAXEDIKSERVICERESPONSE_H
