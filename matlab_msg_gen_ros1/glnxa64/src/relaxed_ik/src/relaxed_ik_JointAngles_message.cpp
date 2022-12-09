// Copyright 2019-2021 The MathWorks, Inc.
// Common copy functions for relaxed_ik/JointAngles
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
#include "relaxed_ik/JointAngles.h"
#include "visibility_control.h"
#include "MATLABROSMsgInterface.hpp"
#include "ROSPubSubTemplates.hpp"
class RELAXED_IK_EXPORT relaxed_ik_msg_JointAngles_common : public MATLABROSMsgInterface<relaxed_ik::JointAngles> {
  public:
    virtual ~relaxed_ik_msg_JointAngles_common(){}
    virtual void copy_from_struct(relaxed_ik::JointAngles* msg, const matlab::data::Struct& arr, MultiLibLoader loader); 
    //----------------------------------------------------------------------------
    virtual MDArray_T get_arr(MDFactory_T& factory, const relaxed_ik::JointAngles* msg, MultiLibLoader loader, size_t size = 1);
};
  void relaxed_ik_msg_JointAngles_common::copy_from_struct(relaxed_ik::JointAngles* msg, const matlab::data::Struct& arr,
               MultiLibLoader loader) {
    try {
        //header
        const matlab::data::StructArray header_arr = arr["Header"];
        auto msgClassPtr_header = getCommonObject<std_msgs::Header>("std_msgs_msg_Header_common",loader);
        msgClassPtr_header->copy_from_struct(&msg->header,header_arr[0],loader);
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'Header' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'Header' is wrong type; expected a struct.");
    }
    try {
        //angles
        const matlab::data::StructArray angles_arr = arr["Angles"];
        for (auto _anglesarr : angles_arr) {
        	std_msgs::Float32 _val;
        auto msgClassPtr_angles = getCommonObject<std_msgs::Float32>("std_msgs_msg_Float32_common",loader);
        msgClassPtr_angles->copy_from_struct(&_val,_anglesarr,loader);
        	msg->angles.push_back(_val);
        }
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'Angles' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'Angles' is wrong type; expected a struct.");
    }
  }
  //----------------------------------------------------------------------------
  MDArray_T relaxed_ik_msg_JointAngles_common::get_arr(MDFactory_T& factory, const relaxed_ik::JointAngles* msg,
       MultiLibLoader loader, size_t size) {
    auto outArray = factory.createStructArray({size,1},{"MessageType","Header","Angles"});
    for(size_t ctr = 0; ctr < size; ctr++){
    outArray[ctr]["MessageType"] = factory.createCharArray("relaxed_ik/JointAngles");
    // header
    auto currentElement_header = (msg + ctr)->header;
    auto msgClassPtr_header = getCommonObject<std_msgs::Header>("std_msgs_msg_Header_common",loader);
    outArray[ctr]["Header"] = msgClassPtr_header->get_arr(factory, &currentElement_header, loader);
    // angles
    auto currentElement_angles = (msg + ctr)->angles;
    auto msgClassPtr_angles = getCommonObject<std_msgs::Float32>("std_msgs_msg_Float32_common",loader);
    outArray[ctr]["Angles"] = msgClassPtr_angles->get_arr(factory,&currentElement_angles[0],loader,currentElement_angles.size());
    }
    return std::move(outArray);
  } 
class RELAXED_IK_EXPORT relaxed_ik_JointAngles_message : public ROSMsgElementInterfaceFactory {
  public:
    virtual ~relaxed_ik_JointAngles_message(){}
    virtual std::shared_ptr<MATLABPublisherInterface> generatePublisherInterface(ElementType type);
    virtual std::shared_ptr<MATLABSubscriberInterface> generateSubscriberInterface(ElementType type);
    virtual std::shared_ptr<MATLABRosbagWriterInterface> generateRosbagWriterInterface(ElementType type);
};  
  std::shared_ptr<MATLABPublisherInterface> 
          relaxed_ik_JointAngles_message::generatePublisherInterface(ElementType type){
    if(type != eMessage){
        throw std::invalid_argument("Wrong input, Expected eMessage");
    }
    return std::make_shared<ROSPublisherImpl<relaxed_ik::JointAngles,relaxed_ik_msg_JointAngles_common>>();
  }
  std::shared_ptr<MATLABSubscriberInterface> 
         relaxed_ik_JointAngles_message::generateSubscriberInterface(ElementType type){
    if(type != eMessage){
        throw std::invalid_argument("Wrong input, Expected eMessage");
    }
    return std::make_shared<ROSSubscriberImpl<relaxed_ik::JointAngles,relaxed_ik::JointAngles::ConstPtr,relaxed_ik_msg_JointAngles_common>>();
  }
#include "ROSbagTemplates.hpp" 
  std::shared_ptr<MATLABRosbagWriterInterface>
         relaxed_ik_JointAngles_message::generateRosbagWriterInterface(ElementType type){
    if(type != eMessage){
        throw std::invalid_argument("Wrong input, Expected eMessage");
    }
    return std::make_shared<ROSBagWriterImpl<relaxed_ik::JointAngles,relaxed_ik_msg_JointAngles_common>>();
  }
#include "register_macro.hpp"
// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
CLASS_LOADER_REGISTER_CLASS(relaxed_ik_msg_JointAngles_common, MATLABROSMsgInterface<relaxed_ik::JointAngles>)
CLASS_LOADER_REGISTER_CLASS(relaxed_ik_JointAngles_message, ROSMsgElementInterfaceFactory)
#ifdef _MSC_VER
#pragma warning(pop)
#else
#pragma GCC diagnostic pop
#endif //_MSC_VER
//gen-1