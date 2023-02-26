// Generated by gencpp from file ob1_arm_hw_interface/armState.msg
// DO NOT EDIT!


#ifndef OB1_ARM_HW_INTERFACE_MESSAGE_ARMSTATE_H
#define OB1_ARM_HW_INTERFACE_MESSAGE_ARMSTATE_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace ob1_arm_hw_interface
{
template <class ContainerAllocator>
struct armState_
{
  typedef armState_<ContainerAllocator> Type;

  armState_()
    : angle()
    , vel()
    , msg_rcv_ctr(0)
    , buffer_health(0)  {
      angle.assign(0.0);

      vel.assign(0.0);
  }
  armState_(const ContainerAllocator& _alloc)
    : angle()
    , vel()
    , msg_rcv_ctr(0)
    , buffer_health(0)  {
  (void)_alloc;
      angle.assign(0.0);

      vel.assign(0.0);
  }



   typedef boost::array<float, 6>  _angle_type;
  _angle_type angle;

   typedef boost::array<float, 6>  _vel_type;
  _vel_type vel;

   typedef int32_t _msg_rcv_ctr_type;
  _msg_rcv_ctr_type msg_rcv_ctr;

   typedef int32_t _buffer_health_type;
  _buffer_health_type buffer_health;





  typedef boost::shared_ptr< ::ob1_arm_hw_interface::armState_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::ob1_arm_hw_interface::armState_<ContainerAllocator> const> ConstPtr;

}; // struct armState_

typedef ::ob1_arm_hw_interface::armState_<std::allocator<void> > armState;

typedef boost::shared_ptr< ::ob1_arm_hw_interface::armState > armStatePtr;
typedef boost::shared_ptr< ::ob1_arm_hw_interface::armState const> armStateConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::ob1_arm_hw_interface::armState_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::ob1_arm_hw_interface::armState_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::ob1_arm_hw_interface::armState_<ContainerAllocator1> & lhs, const ::ob1_arm_hw_interface::armState_<ContainerAllocator2> & rhs)
{
  return lhs.angle == rhs.angle &&
    lhs.vel == rhs.vel &&
    lhs.msg_rcv_ctr == rhs.msg_rcv_ctr &&
    lhs.buffer_health == rhs.buffer_health;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::ob1_arm_hw_interface::armState_<ContainerAllocator1> & lhs, const ::ob1_arm_hw_interface::armState_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace ob1_arm_hw_interface

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::ob1_arm_hw_interface::armState_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::ob1_arm_hw_interface::armState_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::ob1_arm_hw_interface::armState_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::ob1_arm_hw_interface::armState_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::ob1_arm_hw_interface::armState_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::ob1_arm_hw_interface::armState_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::ob1_arm_hw_interface::armState_<ContainerAllocator> >
{
  static const char* value()
  {
    return "ce5e1d1c8004370004acde6f8290eea8";
  }

  static const char* value(const ::ob1_arm_hw_interface::armState_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xce5e1d1c80043700ULL;
  static const uint64_t static_value2 = 0x04acde6f8290eea8ULL;
};

template<class ContainerAllocator>
struct DataType< ::ob1_arm_hw_interface::armState_<ContainerAllocator> >
{
  static const char* value()
  {
    return "ob1_arm_hw_interface/armState";
  }

  static const char* value(const ::ob1_arm_hw_interface::armState_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::ob1_arm_hw_interface::armState_<ContainerAllocator> >
{
  static const char* value()
  {
    return "#Header header \n"
"float32[6] angle # degrees\n"
"float32[6] vel # deg/s\n"
"#time armReadTimestamp \n"
"# time startSyncTime \n"
"# uint32 isrTicks \n"
"int32 msg_rcv_ctr\n"
"int32 buffer_health\n"
;
  }

  static const char* value(const ::ob1_arm_hw_interface::armState_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::ob1_arm_hw_interface::armState_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.angle);
      stream.next(m.vel);
      stream.next(m.msg_rcv_ctr);
      stream.next(m.buffer_health);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct armState_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::ob1_arm_hw_interface::armState_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::ob1_arm_hw_interface::armState_<ContainerAllocator>& v)
  {
    s << indent << "angle[]" << std::endl;
    for (size_t i = 0; i < v.angle.size(); ++i)
    {
      s << indent << "  angle[" << i << "]: ";
      Printer<float>::stream(s, indent + "  ", v.angle[i]);
    }
    s << indent << "vel[]" << std::endl;
    for (size_t i = 0; i < v.vel.size(); ++i)
    {
      s << indent << "  vel[" << i << "]: ";
      Printer<float>::stream(s, indent + "  ", v.vel[i]);
    }
    s << indent << "msg_rcv_ctr: ";
    Printer<int32_t>::stream(s, indent + "  ", v.msg_rcv_ctr);
    s << indent << "buffer_health: ";
    Printer<int32_t>::stream(s, indent + "  ", v.buffer_health);
  }
};

} // namespace message_operations
} // namespace ros

#endif // OB1_ARM_HW_INTERFACE_MESSAGE_ARMSTATE_H
