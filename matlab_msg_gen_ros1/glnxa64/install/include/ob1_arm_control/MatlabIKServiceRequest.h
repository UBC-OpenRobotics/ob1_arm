// Generated by gencpp from file ob1_arm_control/MatlabIKServiceRequest.msg
// DO NOT EDIT!


#ifndef OB1_ARM_CONTROL_MESSAGE_MATLABIKSERVICEREQUEST_H
#define OB1_ARM_CONTROL_MESSAGE_MATLABIKSERVICEREQUEST_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace ob1_arm_control
{
template <class ContainerAllocator>
struct MatlabIKServiceRequest_
{
  typedef MatlabIKServiceRequest_<ContainerAllocator> Type;

  MatlabIKServiceRequest_()
    : pose_target()
    , tolerance()  {
    }
  MatlabIKServiceRequest_(const ContainerAllocator& _alloc)
    : pose_target(_alloc)
    , tolerance(_alloc)  {
  (void)_alloc;
    }



   typedef std::vector<double, typename ContainerAllocator::template rebind<double>::other >  _pose_target_type;
  _pose_target_type pose_target;

   typedef std::vector<double, typename ContainerAllocator::template rebind<double>::other >  _tolerance_type;
  _tolerance_type tolerance;





  typedef boost::shared_ptr< ::ob1_arm_control::MatlabIKServiceRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::ob1_arm_control::MatlabIKServiceRequest_<ContainerAllocator> const> ConstPtr;

}; // struct MatlabIKServiceRequest_

typedef ::ob1_arm_control::MatlabIKServiceRequest_<std::allocator<void> > MatlabIKServiceRequest;

typedef boost::shared_ptr< ::ob1_arm_control::MatlabIKServiceRequest > MatlabIKServiceRequestPtr;
typedef boost::shared_ptr< ::ob1_arm_control::MatlabIKServiceRequest const> MatlabIKServiceRequestConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::ob1_arm_control::MatlabIKServiceRequest_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::ob1_arm_control::MatlabIKServiceRequest_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::ob1_arm_control::MatlabIKServiceRequest_<ContainerAllocator1> & lhs, const ::ob1_arm_control::MatlabIKServiceRequest_<ContainerAllocator2> & rhs)
{
  return lhs.pose_target == rhs.pose_target &&
    lhs.tolerance == rhs.tolerance;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::ob1_arm_control::MatlabIKServiceRequest_<ContainerAllocator1> & lhs, const ::ob1_arm_control::MatlabIKServiceRequest_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace ob1_arm_control

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::ob1_arm_control::MatlabIKServiceRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::ob1_arm_control::MatlabIKServiceRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::ob1_arm_control::MatlabIKServiceRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::ob1_arm_control::MatlabIKServiceRequest_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::ob1_arm_control::MatlabIKServiceRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::ob1_arm_control::MatlabIKServiceRequest_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::ob1_arm_control::MatlabIKServiceRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "ad09fc55b0d494343b290d861e0efa46";
  }

  static const char* value(const ::ob1_arm_control::MatlabIKServiceRequest_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xad09fc55b0d49434ULL;
  static const uint64_t static_value2 = 0x3b290d861e0efa46ULL;
};

template<class ContainerAllocator>
struct DataType< ::ob1_arm_control::MatlabIKServiceRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "ob1_arm_control/MatlabIKServiceRequest";
  }

  static const char* value(const ::ob1_arm_control::MatlabIKServiceRequest_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::ob1_arm_control::MatlabIKServiceRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "float64[] pose_target\n"
"float64[] tolerance\n"
;
  }

  static const char* value(const ::ob1_arm_control::MatlabIKServiceRequest_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::ob1_arm_control::MatlabIKServiceRequest_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.pose_target);
      stream.next(m.tolerance);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct MatlabIKServiceRequest_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::ob1_arm_control::MatlabIKServiceRequest_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::ob1_arm_control::MatlabIKServiceRequest_<ContainerAllocator>& v)
  {
    s << indent << "pose_target[]" << std::endl;
    for (size_t i = 0; i < v.pose_target.size(); ++i)
    {
      s << indent << "  pose_target[" << i << "]: ";
      Printer<double>::stream(s, indent + "  ", v.pose_target[i]);
    }
    s << indent << "tolerance[]" << std::endl;
    for (size_t i = 0; i < v.tolerance.size(); ++i)
    {
      s << indent << "  tolerance[" << i << "]: ";
      Printer<double>::stream(s, indent + "  ", v.tolerance[i]);
    }
  }
};

} // namespace message_operations
} // namespace ros

#endif // OB1_ARM_CONTROL_MESSAGE_MATLABIKSERVICEREQUEST_H
