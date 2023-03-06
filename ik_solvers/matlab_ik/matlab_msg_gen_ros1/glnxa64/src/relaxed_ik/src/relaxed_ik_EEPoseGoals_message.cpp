// Copyright 2019-2021 The MathWorks, Inc.
// Common copy functions for relaxed_ik/EEPoseGoals
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
#include "relaxed_ik/EEPoseGoals.h"
#include "visibility_control.h"
#include "MATLABROSMsgInterface.hpp"
#include "ROSPubSubTemplates.hpp"
class RELAXED_IK_EXPORT relaxed_ik_msg_EEPoseGoals_common : public MATLABROSMsgInterface<relaxed_ik::EEPoseGoals> {
  public:
    virtual ~relaxed_ik_msg_EEPoseGoals_common(){}
    virtual void copy_from_struct(relaxed_ik::EEPoseGoals* msg, const matlab::data::Struct& arr, MultiLibLoader loader); 
    //----------------------------------------------------------------------------
    virtual MDArray_T get_arr(MDFactory_T& factory, const relaxed_ik::EEPoseGoals* msg, MultiLibLoader loader, size_t size = 1);
};
  void relaxed_ik_msg_EEPoseGoals_common::copy_from_struct(relaxed_ik::EEPoseGoals* msg, const matlab::data::Struct& arr,
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
        //ee_poses
        const matlab::data::StructArray ee_poses_arr = arr["EePoses"];
        for (auto _ee_posesarr : ee_poses_arr) {
        	geometry_msgs::Pose _val;
        auto msgClassPtr_ee_poses = getCommonObject<geometry_msgs::Pose>("geometry_msgs_msg_Pose_common",loader);
        msgClassPtr_ee_poses->copy_from_struct(&_val,_ee_posesarr,loader);
        	msg->ee_poses.push_back(_val);
        }
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'EePoses' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'EePoses' is wrong type; expected a struct.");
    }
  }
  //----------------------------------------------------------------------------
  MDArray_T relaxed_ik_msg_EEPoseGoals_common::get_arr(MDFactory_T& factory, const relaxed_ik::EEPoseGoals* msg,
       MultiLibLoader loader, size_t size) {
    auto outArray = factory.createStructArray({size,1},{"MessageType","Header","EePoses"});
    for(size_t ctr = 0; ctr < size; ctr++){
    outArray[ctr]["MessageType"] = factory.createCharArray("relaxed_ik/EEPoseGoals");
    // header
    auto currentElement_header = (msg + ctr)->header;
    auto msgClassPtr_header = getCommonObject<std_msgs::Header>("std_msgs_msg_Header_common",loader);
    outArray[ctr]["Header"] = msgClassPtr_header->get_arr(factory, &currentElement_header, loader);
    // ee_poses
    auto currentElement_ee_poses = (msg + ctr)->ee_poses;
    auto msgClassPtr_ee_poses = getCommonObject<geometry_msgs::Pose>("geometry_msgs_msg_Pose_common",loader);
    outArray[ctr]["EePoses"] = msgClassPtr_ee_poses->get_arr(factory,&currentElement_ee_poses[0],loader,currentElement_ee_poses.size());
    }
    return std::move(outArray);
  } 
class RELAXED_IK_EXPORT relaxed_ik_EEPoseGoals_message : public ROSMsgElementInterfaceFactory {
  public:
    virtual ~relaxed_ik_EEPoseGoals_message(){}
    virtual std::shared_ptr<MATLABPublisherInterface> generatePublisherInterface(ElementType type);
    virtual std::shared_ptr<MATLABSubscriberInterface> generateSubscriberInterface(ElementType type);
    virtual std::shared_ptr<MATLABRosbagWriterInterface> generateRosbagWriterInterface(ElementType type);
};  
  std::shared_ptr<MATLABPublisherInterface> 
          relaxed_ik_EEPoseGoals_message::generatePublisherInterface(ElementType type){
    if(type != eMessage){
        throw std::invalid_argument("Wrong input, Expected eMessage");
    }
    return std::make_shared<ROSPublisherImpl<relaxed_ik::EEPoseGoals,relaxed_ik_msg_EEPoseGoals_common>>();
  }
  std::shared_ptr<MATLABSubscriberInterface> 
         relaxed_ik_EEPoseGoals_message::generateSubscriberInterface(ElementType type){
    if(type != eMessage){
        throw std::invalid_argument("Wrong input, Expected eMessage");
    }
    return std::make_shared<ROSSubscriberImpl<relaxed_ik::EEPoseGoals,relaxed_ik::EEPoseGoals::ConstPtr,relaxed_ik_msg_EEPoseGoals_common>>();
  }
#include "ROSbagTemplates.hpp" 
  std::shared_ptr<MATLABRosbagWriterInterface>
         relaxed_ik_EEPoseGoals_message::generateRosbagWriterInterface(ElementType type){
    if(type != eMessage){
        throw std::invalid_argument("Wrong input, Expected eMessage");
    }
    return std::make_shared<ROSBagWriterImpl<relaxed_ik::EEPoseGoals,relaxed_ik_msg_EEPoseGoals_common>>();
  }
#include "register_macro.hpp"
// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
CLASS_LOADER_REGISTER_CLASS(relaxed_ik_msg_EEPoseGoals_common, MATLABROSMsgInterface<relaxed_ik::EEPoseGoals>)
CLASS_LOADER_REGISTER_CLASS(relaxed_ik_EEPoseGoals_message, ROSMsgElementInterfaceFactory)
#ifdef _MSC_VER
#pragma warning(pop)
#else
#pragma GCC diagnostic pop
#endif //_MSC_VER
//gen-1