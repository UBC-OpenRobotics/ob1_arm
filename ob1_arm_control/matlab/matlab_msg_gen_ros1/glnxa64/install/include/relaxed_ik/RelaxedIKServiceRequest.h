// Generated by gencpp from file relaxed_ik/RelaxedIKServiceRequest.msg
// DO NOT EDIT!


#ifndef RELAXED_IK_MESSAGE_RELAXEDIKSERVICEREQUEST_H
#define RELAXED_IK_MESSAGE_RELAXEDIKSERVICEREQUEST_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <relaxed_ik/EEPoseGoals.h>

namespace relaxed_ik
{
template <class ContainerAllocator>
struct RelaxedIKServiceRequest_
{
  typedef RelaxedIKServiceRequest_<ContainerAllocator> Type;

  RelaxedIKServiceRequest_()
    : pose_goals()  {
    }
  RelaxedIKServiceRequest_(const ContainerAllocator& _alloc)
    : pose_goals(_alloc)  {
  (void)_alloc;
    }



   typedef  ::relaxed_ik::EEPoseGoals_<ContainerAllocator>  _pose_goals_type;
  _pose_goals_type pose_goals;





  typedef boost::shared_ptr< ::relaxed_ik::RelaxedIKServiceRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::relaxed_ik::RelaxedIKServiceRequest_<ContainerAllocator> const> ConstPtr;

}; // struct RelaxedIKServiceRequest_

typedef ::relaxed_ik::RelaxedIKServiceRequest_<std::allocator<void> > RelaxedIKServiceRequest;

typedef boost::shared_ptr< ::relaxed_ik::RelaxedIKServiceRequest > RelaxedIKServiceRequestPtr;
typedef boost::shared_ptr< ::relaxed_ik::RelaxedIKServiceRequest const> RelaxedIKServiceRequestConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::relaxed_ik::RelaxedIKServiceRequest_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::relaxed_ik::RelaxedIKServiceRequest_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::relaxed_ik::RelaxedIKServiceRequest_<ContainerAllocator1> & lhs, const ::relaxed_ik::RelaxedIKServiceRequest_<ContainerAllocator2> & rhs)
{
  return lhs.pose_goals == rhs.pose_goals;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::relaxed_ik::RelaxedIKServiceRequest_<ContainerAllocator1> & lhs, const ::relaxed_ik::RelaxedIKServiceRequest_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace relaxed_ik

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::relaxed_ik::RelaxedIKServiceRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::relaxed_ik::RelaxedIKServiceRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::relaxed_ik::RelaxedIKServiceRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::relaxed_ik::RelaxedIKServiceRequest_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::relaxed_ik::RelaxedIKServiceRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::relaxed_ik::RelaxedIKServiceRequest_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::relaxed_ik::RelaxedIKServiceRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "1fa9b22ae31214968e5d110b292dc65e";
  }

  static const char* value(const ::relaxed_ik::RelaxedIKServiceRequest_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x1fa9b22ae3121496ULL;
  static const uint64_t static_value2 = 0x8e5d110b292dc65eULL;
};

template<class ContainerAllocator>
struct DataType< ::relaxed_ik::RelaxedIKServiceRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "relaxed_ik/RelaxedIKServiceRequest";
  }

  static const char* value(const ::relaxed_ik::RelaxedIKServiceRequest_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::relaxed_ik::RelaxedIKServiceRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "EEPoseGoals pose_goals\n"
"\n"
"================================================================================\n"
"MSG: relaxed_ik/EEPoseGoals\n"
"std_msgs/Header header\n"
"geometry_msgs/Pose[] ee_poses\n"
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
"MSG: geometry_msgs/Pose\n"
"# A representation of pose in free space, composed of position and orientation. \n"
"Point position\n"
"Quaternion orientation\n"
"\n"
"================================================================================\n"
"MSG: geometry_msgs/Point\n"
"# This contains the position of a point in free space\n"
"float64 x\n"
"float64 y\n"
"float64 z\n"
"\n"
"================================================================================\n"
"MSG: geometry_msgs/Quaternion\n"
"# This represents an orientation in free space in quaternion form.\n"
"\n"
"float64 x\n"
"float64 y\n"
"float64 z\n"
"float64 w\n"
;
  }

  static const char* value(const ::relaxed_ik::RelaxedIKServiceRequest_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::relaxed_ik::RelaxedIKServiceRequest_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.pose_goals);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct RelaxedIKServiceRequest_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::relaxed_ik::RelaxedIKServiceRequest_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::relaxed_ik::RelaxedIKServiceRequest_<ContainerAllocator>& v)
  {
    s << indent << "pose_goals: ";
    s << std::endl;
    Printer< ::relaxed_ik::EEPoseGoals_<ContainerAllocator> >::stream(s, indent + "  ", v.pose_goals);
  }
};

} // namespace message_operations
} // namespace ros

#endif // RELAXED_IK_MESSAGE_RELAXEDIKSERVICEREQUEST_H