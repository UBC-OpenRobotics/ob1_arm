// Copyright 2019-2021 The MathWorks, Inc.
// Common copy functions for ob1_arm_hw_interface/armState
#include "boost/date_time.hpp"
#include "boost/shared_array.hpp"
#ifdef _MSC_VER
#pragma warning(push)
#pragma warning(disable : 4244)
#pragma warning(disable : 4265)
#pragma warning(disable : 4458)
#pragma warning(disable : 4100)
#pragma warning(disable : 4127)
#pragma warning(disable : 4267)
#pragma warning(disable : 4068)
#pragma warning(disable : 4245)
#else
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
#pragma GCC diagnostic ignored "-Wunused-local-typedefs"
#pragma GCC diagnostic ignored "-Wredundant-decls"
#pragma GCC diagnostic ignored "-Wnon-virtual-dtor"
#pragma GCC diagnostic ignored "-Wdelete-non-virtual-dtor"
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wunused-variable"
#pragma GCC diagnostic ignored "-Wshadow"
#endif //_MSC_VER
#include "ros/ros.h"
#include "ob1_arm_hw_interface/armState.h"
#include "visibility_control.h"
#include "MATLABROSMsgInterface.hpp"
#include "ROSPubSubTemplates.hpp"
class OB1_ARM_HW_INTERFACE_EXPORT ob1_arm_hw_interface_msg_armState_common : public MATLABROSMsgInterface<ob1_arm_hw_interface::armState> {
  public:
    virtual ~ob1_arm_hw_interface_msg_armState_common(){}
    virtual void copy_from_struct(ob1_arm_hw_interface::armState* msg, const matlab::data::Struct& arr, MultiLibLoader loader); 
    //----------------------------------------------------------------------------
    virtual MDArray_T get_arr(MDFactory_T& factory, const ob1_arm_hw_interface::armState* msg, MultiLibLoader loader, size_t size = 1);
};
  void ob1_arm_hw_interface_msg_armState_common::copy_from_struct(ob1_arm_hw_interface::armState* msg, const matlab::data::Struct& arr,
               MultiLibLoader loader) {
    try {
        //angle
        const matlab::data::TypedArray<float> angle_arr = arr["Angle"];
        size_t nelem = 6;
        	std::copy(angle_arr.begin(), angle_arr.begin()+nelem, msg->angle.begin());
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'Angle' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'Angle' is wrong type; expected a single.");
    }
    try {
        //vel
        const matlab::data::TypedArray<float> vel_arr = arr["Vel"];
        size_t nelem = 6;
        	std::copy(vel_arr.begin(), vel_arr.begin()+nelem, msg->vel.begin());
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'Vel' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'Vel' is wrong type; expected a single.");
    }
    try {
        //msg_rcv_ctr
        const matlab::data::TypedArray<int32_t> msg_rcv_ctr_arr = arr["MsgRcvCtr"];
        msg->msg_rcv_ctr = msg_rcv_ctr_arr[0];
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'MsgRcvCtr' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'MsgRcvCtr' is wrong type; expected a int32.");
    }
    try {
        //buffer_health
        const matlab::data::TypedArray<int32_t> buffer_health_arr = arr["BufferHealth"];
        msg->buffer_health = buffer_health_arr[0];
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'BufferHealth' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'BufferHealth' is wrong type; expected a int32.");
    }
  }
  //----------------------------------------------------------------------------
  MDArray_T ob1_arm_hw_interface_msg_armState_common::get_arr(MDFactory_T& factory, const ob1_arm_hw_interface::armState* msg,
       MultiLibLoader loader, size_t size) {
    auto outArray = factory.createStructArray({size,1},{"MessageType","Angle","Vel","MsgRcvCtr","BufferHealth"});
    for(size_t ctr = 0; ctr < size; ctr++){
    outArray[ctr]["MessageType"] = factory.createCharArray("ob1_arm_hw_interface/armState");
    // angle
    auto currentElement_angle = (msg + ctr)->angle;
    outArray[ctr]["Angle"] = factory.createArray<ob1_arm_hw_interface::armState::_angle_type::const_iterator, float>({currentElement_angle.size(),1}, currentElement_angle.begin(), currentElement_angle.end());
    // vel
    auto currentElement_vel = (msg + ctr)->vel;
    outArray[ctr]["Vel"] = factory.createArray<ob1_arm_hw_interface::armState::_vel_type::const_iterator, float>({currentElement_vel.size(),1}, currentElement_vel.begin(), currentElement_vel.end());
    // msg_rcv_ctr
    auto currentElement_msg_rcv_ctr = (msg + ctr)->msg_rcv_ctr;
    outArray[ctr]["MsgRcvCtr"] = factory.createScalar(currentElement_msg_rcv_ctr);
    // buffer_health
    auto currentElement_buffer_health = (msg + ctr)->buffer_health;
    outArray[ctr]["BufferHealth"] = factory.createScalar(currentElement_buffer_health);
    }
    return std::move(outArray);
  } 
class OB1_ARM_HW_INTERFACE_EXPORT ob1_arm_hw_interface_armState_message : public ROSMsgElementInterfaceFactory {
  public:
    virtual ~ob1_arm_hw_interface_armState_message(){}
    virtual std::shared_ptr<MATLABPublisherInterface> generatePublisherInterface(ElementType type);
    virtual std::shared_ptr<MATLABSubscriberInterface> generateSubscriberInterface(ElementType type);
    virtual std::shared_ptr<MATLABRosbagWriterInterface> generateRosbagWriterInterface(ElementType type);
};  
  std::shared_ptr<MATLABPublisherInterface> 
          ob1_arm_hw_interface_armState_message::generatePublisherInterface(ElementType type){
    if(type != eMessage){
        throw std::invalid_argument("Wrong input, Expected eMessage");
    }
    return std::make_shared<ROSPublisherImpl<ob1_arm_hw_interface::armState,ob1_arm_hw_interface_msg_armState_common>>();
  }
  std::shared_ptr<MATLABSubscriberInterface> 
         ob1_arm_hw_interface_armState_message::generateSubscriberInterface(ElementType type){
    if(type != eMessage){
        throw std::invalid_argument("Wrong input, Expected eMessage");
    }
    return std::make_shared<ROSSubscriberImpl<ob1_arm_hw_interface::armState,ob1_arm_hw_interface::armState::ConstPtr,ob1_arm_hw_interface_msg_armState_common>>();
  }
#include "ROSbagTemplates.hpp" 
  std::shared_ptr<MATLABRosbagWriterInterface>
         ob1_arm_hw_interface_armState_message::generateRosbagWriterInterface(ElementType type){
    if(type != eMessage){
        throw std::invalid_argument("Wrong input, Expected eMessage");
    }
    return std::make_shared<ROSBagWriterImpl<ob1_arm_hw_interface::armState,ob1_arm_hw_interface_msg_armState_common>>();
  }
#include "register_macro.hpp"
// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
CLASS_LOADER_REGISTER_CLASS(ob1_arm_hw_interface_msg_armState_common, MATLABROSMsgInterface<ob1_arm_hw_interface::armState>)
CLASS_LOADER_REGISTER_CLASS(ob1_arm_hw_interface_armState_message, ROSMsgElementInterfaceFactory)
#ifdef _MSC_VER
#pragma warning(pop)
#else
#pragma GCC diagnostic pop
#endif //_MSC_VER
//gen-1