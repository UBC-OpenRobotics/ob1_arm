// Generated by gencpp from file relaxed_ik/EEPoseGoals.msg
// DO NOT EDIT!


#ifndef RELAXED_IK_MESSAGE_EEPOSEGOALS_H
#define RELAXED_IK_MESSAGE_EEPOSEGOALS_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <std_msgs/Header.h>
#include <geometry_msgs/Pose.h>

namespace relaxed_ik
{
template <class ContainerAllocator>
struct EEPoseGoals_
{
  typedef EEPoseGoals_<ContainerAllocator> Type;

  EEPoseGoals_()
    : header()
    , ee_poses()  {
    }
  EEPoseGoals_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , ee_poses(_alloc)  {
  (void)_alloc;
    }



   typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef std::vector< ::geometry_msgs::Pose_<ContainerAllocator> , typename ContainerAllocator::template rebind< ::geometry_msgs::Pose_<ContainerAllocator> >::other >  _ee_poses_type;
  _ee_poses_type ee_poses;





  typedef boost::shared_ptr< ::relaxed_ik::EEPoseGoals_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::relaxed_ik::EEPoseGoals_<ContainerAllocator> const> ConstPtr;

}; // struct EEPoseGoals_

typedef ::relaxed_ik::EEPoseGoals_<std::allocator<void> > EEPoseGoals;

typedef boost::shared_ptr< ::relaxed_ik::EEPoseGoals > EEPoseGoalsPtr;
typedef boost::shared_ptr< ::relaxed_ik::EEPoseGoals const> EEPoseGoalsConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::relaxed_ik::EEPoseGoals_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::relaxed_ik::EEPoseGoals_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::relaxed_ik::EEPoseGoals_<ContainerAllocator1> & lhs, const ::relaxed_ik::EEPoseGoals_<ContainerAllocator2> & rhs)
{
  return lhs.header == rhs.header &&
    lhs.ee_poses == rhs.ee_poses;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::relaxed_ik::EEPoseGoals_<ContainerAllocator1> & lhs, const ::relaxed_ik::EEPoseGoals_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace relaxed_ik

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::relaxed_ik::EEPoseGoals_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::relaxed_ik::EEPoseGoals_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::relaxed_ik::EEPoseGoals_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::relaxed_ik::EEPoseGoals_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::relaxed_ik::EEPoseGoals_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::relaxed_ik::EEPoseGoals_<ContainerAllocator> const>
  : TrueType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::relaxed_ik::EEPoseGoals_<ContainerAllocator> >
{
  static const char* value()
  {
    return "db34972df908bdbeb860c627c17f6b1e";
  }

  static const char* value(const ::relaxed_ik::EEPoseGoals_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xdb34972df908bdbeULL;
  static const uint64_t static_value2 = 0xb860c627c17f6b1eULL;
};

template<class ContainerAllocator>
struct DataType< ::relaxed_ik::EEPoseGoals_<ContainerAllocator> >
{
  static const char* value()
  {
    return "relaxed_ik/EEPoseGoals";
  }

  static const char* value(const ::relaxed_ik::EEPoseGoals_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::relaxed_ik::EEPoseGoals_<ContainerAllocator> >
{
  static const char* value()
  {
    return "std_msgs/Header header\n"
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

  static const char* value(const ::relaxed_ik::EEPoseGoals_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::relaxed_ik::EEPoseGoals_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.header);
      stream.next(m.ee_poses);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct EEPoseGoals_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::relaxed_ik::EEPoseGoals_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::relaxed_ik::EEPoseGoals_<ContainerAllocator>& v)
  {
    s << indent << "header: ";
    s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "ee_poses[]" << std::endl;
    for (size_t i = 0; i < v.ee_poses.size(); ++i)
    {
      s << indent << "  ee_poses[" << i << "]: ";
      s << std::endl;
      s << indent;
      Printer< ::geometry_msgs::Pose_<ContainerAllocator> >::stream(s, indent + "    ", v.ee_poses[i]);
    }
  }
};

} // namespace message_operations
} // namespace ros

#endif // RELAXED_IK_MESSAGE_EEPOSEGOALS_H
