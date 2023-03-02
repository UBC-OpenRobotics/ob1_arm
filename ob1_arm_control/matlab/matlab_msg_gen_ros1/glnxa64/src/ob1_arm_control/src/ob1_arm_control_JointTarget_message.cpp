// Copyright 2019-2021 The MathWorks, Inc.
// Common copy functions for ob1_arm_control/JointTarget
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
#include "ob1_arm_control/JointTarget.h"
#include "visibility_control.h"
#include "MATLABROSMsgInterface.hpp"
#include "ROSPubSubTemplates.hpp"
class OB1_ARM_CONTROL_EXPORT ob1_arm_control_msg_JointTarget_common : public MATLABROSMsgInterface<ob1_arm_control::JointTarget> {
  public:
    virtual ~ob1_arm_control_msg_JointTarget_common(){}
    virtual void copy_from_struct(ob1_arm_control::JointTarget* msg, const matlab::data::Struct& arr, MultiLibLoader loader); 
    //----------------------------------------------------------------------------
    virtual MDArray_T get_arr(MDFactory_T& factory, const ob1_arm_control::JointTarget* msg, MultiLibLoader loader, size_t size = 1);
};
  void ob1_arm_control_msg_JointTarget_common::copy_from_struct(ob1_arm_control::JointTarget* msg, const matlab::data::Struct& arr,
               MultiLibLoader loader) {
    try {
        //joint_target
        const matlab::data::TypedArray<double> joint_target_arr = arr["JointTarget_"];
        size_t nelem = joint_target_arr.getNumberOfElements();
        	msg->joint_target.resize(nelem);
        	std::copy(joint_target_arr.begin(), joint_target_arr.begin()+nelem, msg->joint_target.begin());
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'JointTarget_' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'JointTarget_' is wrong type; expected a double.");
    }
  }
  //----------------------------------------------------------------------------
  MDArray_T ob1_arm_control_msg_JointTarget_common::get_arr(MDFactory_T& factory, const ob1_arm_control::JointTarget* msg,
       MultiLibLoader loader, size_t size) {
    auto outArray = factory.createStructArray({size,1},{"MessageType","JointTarget_"});
    for(size_t ctr = 0; ctr < size; ctr++){
    outArray[ctr]["MessageType"] = factory.createCharArray("ob1_arm_control/JointTarget");
    // joint_target
    auto currentElement_joint_target = (msg + ctr)->joint_target;
    outArray[ctr]["JointTarget_"] = factory.createArray<ob1_arm_control::JointTarget::_joint_target_type::const_iterator, double>({currentElement_joint_target.size(),1}, currentElement_joint_target.begin(), currentElement_joint_target.end());
    }
    return std::move(outArray);
  } 
class OB1_ARM_CONTROL_EXPORT ob1_arm_control_JointTarget_message : public ROSMsgElementInterfaceFactory {
  public:
    virtual ~ob1_arm_control_JointTarget_message(){}
    virtual std::shared_ptr<MATLABPublisherInterface> generatePublisherInterface(ElementType type);
    virtual std::shared_ptr<MATLABSubscriberInterface> generateSubscriberInterface(ElementType type);
    virtual std::shared_ptr<MATLABRosbagWriterInterface> generateRosbagWriterInterface(ElementType type);
};  
  std::shared_ptr<MATLABPublisherInterface> 
          ob1_arm_control_JointTarget_message::generatePublisherInterface(ElementType type){
    if(type != eMessage){
        throw std::invalid_argument("Wrong input, Expected eMessage");
    }
    return std::make_shared<ROSPublisherImpl<ob1_arm_control::JointTarget,ob1_arm_control_msg_JointTarget_common>>();
  }
  std::shared_ptr<MATLABSubscriberInterface> 
         ob1_arm_control_JointTarget_message::generateSubscriberInterface(ElementType type){
    if(type != eMessage){
        throw std::invalid_argument("Wrong input, Expected eMessage");
    }
    return std::make_shared<ROSSubscriberImpl<ob1_arm_control::JointTarget,ob1_arm_control::JointTarget::ConstPtr,ob1_arm_control_msg_JointTarget_common>>();
  }
#include "ROSbagTemplates.hpp" 
  std::shared_ptr<MATLABRosbagWriterInterface>
         ob1_arm_control_JointTarget_message::generateRosbagWriterInterface(ElementType type){
    if(type != eMessage){
        throw std::invalid_argument("Wrong input, Expected eMessage");
    }
    return std::make_shared<ROSBagWriterImpl<ob1_arm_control::JointTarget,ob1_arm_control_msg_JointTarget_common>>();
  }
#include "register_macro.hpp"
// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
CLASS_LOADER_REGISTER_CLASS(ob1_arm_control_msg_JointTarget_common, MATLABROSMsgInterface<ob1_arm_control::JointTarget>)
CLASS_LOADER_REGISTER_CLASS(ob1_arm_control_JointTarget_message, ROSMsgElementInterfaceFactory)
#ifdef _MSC_VER
#pragma warning(pop)
#else
#pragma GCC diagnostic pop
#endif //_MSC_VER
//gen-1