// Copyright 2019-2021 The MathWorks, Inc.
// Common copy functions for relaxed_ik/RelaxedIKServiceRequest
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
#include "relaxed_ik/RelaxedIKService.h"
#include "visibility_control.h"
#include "ROSPubSubTemplates.hpp"
#include "ROSServiceTemplates.hpp"
class RELAXED_IK_EXPORT relaxed_ik_msg_RelaxedIKServiceRequest_common : public MATLABROSMsgInterface<relaxed_ik::RelaxedIKService::Request> {
  public:
    virtual ~relaxed_ik_msg_RelaxedIKServiceRequest_common(){}
    virtual void copy_from_struct(relaxed_ik::RelaxedIKService::Request* msg, const matlab::data::Struct& arr, MultiLibLoader loader); 
    //----------------------------------------------------------------------------
    virtual MDArray_T get_arr(MDFactory_T& factory, const relaxed_ik::RelaxedIKService::Request* msg, MultiLibLoader loader, size_t size = 1);
};
  void relaxed_ik_msg_RelaxedIKServiceRequest_common::copy_from_struct(relaxed_ik::RelaxedIKService::Request* msg, const matlab::data::Struct& arr,
               MultiLibLoader loader) {
    try {
        //pose_goals
        const matlab::data::StructArray pose_goals_arr = arr["PoseGoals"];
        auto msgClassPtr_pose_goals = getCommonObject<relaxed_ik::EEPoseGoals>("relaxed_ik_msg_EEPoseGoals_common",loader);
        msgClassPtr_pose_goals->copy_from_struct(&msg->pose_goals,pose_goals_arr[0],loader);
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'PoseGoals' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'PoseGoals' is wrong type; expected a struct.");
    }
  }
  //----------------------------------------------------------------------------
  MDArray_T relaxed_ik_msg_RelaxedIKServiceRequest_common::get_arr(MDFactory_T& factory, const relaxed_ik::RelaxedIKService::Request* msg,
       MultiLibLoader loader, size_t size) {
    auto outArray = factory.createStructArray({size,1},{"MessageType","PoseGoals"});
    for(size_t ctr = 0; ctr < size; ctr++){
    outArray[ctr]["MessageType"] = factory.createCharArray("relaxed_ik/RelaxedIKServiceRequest");
    // pose_goals
    auto currentElement_pose_goals = (msg + ctr)->pose_goals;
    auto msgClassPtr_pose_goals = getCommonObject<relaxed_ik::EEPoseGoals>("relaxed_ik_msg_EEPoseGoals_common",loader);
    outArray[ctr]["PoseGoals"] = msgClassPtr_pose_goals->get_arr(factory, &currentElement_pose_goals, loader);
    }
    return std::move(outArray);
  }
class RELAXED_IK_EXPORT relaxed_ik_msg_RelaxedIKServiceResponse_common : public MATLABROSMsgInterface<relaxed_ik::RelaxedIKService::Response> {
  public:
    virtual ~relaxed_ik_msg_RelaxedIKServiceResponse_common(){}
    virtual void copy_from_struct(relaxed_ik::RelaxedIKService::Response* msg, const matlab::data::Struct& arr, MultiLibLoader loader); 
    //----------------------------------------------------------------------------
    virtual MDArray_T get_arr(MDFactory_T& factory, const relaxed_ik::RelaxedIKService::Response* msg, MultiLibLoader loader, size_t size = 1);
};
  void relaxed_ik_msg_RelaxedIKServiceResponse_common::copy_from_struct(relaxed_ik::RelaxedIKService::Response* msg, const matlab::data::Struct& arr,
               MultiLibLoader loader) {
    try {
        //joint_angles
        const matlab::data::StructArray joint_angles_arr = arr["JointAngles"];
        auto msgClassPtr_joint_angles = getCommonObject<relaxed_ik::JointAngles>("relaxed_ik_msg_JointAngles_common",loader);
        msgClassPtr_joint_angles->copy_from_struct(&msg->joint_angles,joint_angles_arr[0],loader);
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'JointAngles' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'JointAngles' is wrong type; expected a struct.");
    }
  }
  //----------------------------------------------------------------------------
  MDArray_T relaxed_ik_msg_RelaxedIKServiceResponse_common::get_arr(MDFactory_T& factory, const relaxed_ik::RelaxedIKService::Response* msg,
       MultiLibLoader loader, size_t size) {
    auto outArray = factory.createStructArray({size,1},{"MessageType","JointAngles"});
    for(size_t ctr = 0; ctr < size; ctr++){
    outArray[ctr]["MessageType"] = factory.createCharArray("relaxed_ik/RelaxedIKServiceResponse");
    // joint_angles
    auto currentElement_joint_angles = (msg + ctr)->joint_angles;
    auto msgClassPtr_joint_angles = getCommonObject<relaxed_ik::JointAngles>("relaxed_ik_msg_JointAngles_common",loader);
    outArray[ctr]["JointAngles"] = msgClassPtr_joint_angles->get_arr(factory, &currentElement_joint_angles, loader);
    }
    return std::move(outArray);
  } 
class RELAXED_IK_EXPORT relaxed_ik_RelaxedIKService_service : public ROSMsgElementInterfaceFactory {
  public:
    virtual ~relaxed_ik_RelaxedIKService_service(){}
    virtual std::shared_ptr<MATLABPublisherInterface> generatePublisherInterface(ElementType type);
    virtual std::shared_ptr<MATLABSubscriberInterface> generateSubscriberInterface(ElementType type);
    virtual std::shared_ptr<MATLABRosbagWriterInterface> generateRosbagWriterInterface(ElementType type);
    virtual std::shared_ptr<MATLABSvcServerInterface> generateSvcServerInterface();
    virtual std::shared_ptr<MATLABSvcClientInterface> generateSvcClientInterface();
};  
  std::shared_ptr<MATLABPublisherInterface> 
          relaxed_ik_RelaxedIKService_service::generatePublisherInterface(ElementType type){
    std::shared_ptr<MATLABPublisherInterface> ptr;
    if(type == eRequest){
        ptr = std::make_shared<ROSPublisherImpl<relaxed_ik::RelaxedIKService::Request,relaxed_ik_msg_RelaxedIKServiceRequest_common>>();
    }else if(type == eResponse){
        ptr = std::make_shared<ROSPublisherImpl<relaxed_ik::RelaxedIKService::Response,relaxed_ik_msg_RelaxedIKServiceResponse_common>>();
    }else{
        throw std::invalid_argument("Wrong input, Expected 'Request' or 'Response'");
    }
    return ptr;
  }
  std::shared_ptr<MATLABSubscriberInterface> 
          relaxed_ik_RelaxedIKService_service::generateSubscriberInterface(ElementType type){
    std::shared_ptr<MATLABSubscriberInterface> ptr;
    if(type == eRequest){
        ptr = std::make_shared<ROSSubscriberImpl<relaxed_ik::RelaxedIKService::Request,relaxed_ik::RelaxedIKService::Request::ConstPtr,relaxed_ik_msg_RelaxedIKServiceRequest_common>>();
    }else if(type == eResponse){
        ptr = std::make_shared<ROSSubscriberImpl<relaxed_ik::RelaxedIKService::Response,relaxed_ik::RelaxedIKService::Response::ConstPtr,relaxed_ik_msg_RelaxedIKServiceResponse_common>>();
    }else{
        throw std::invalid_argument("Wrong input, Expected 'Request' or 'Response'");
    }
    return ptr;
  }
  std::shared_ptr<MATLABSvcServerInterface> 
          relaxed_ik_RelaxedIKService_service::generateSvcServerInterface(){
    return std::make_shared<ROSSvcServerImpl<relaxed_ik::RelaxedIKService::Request,relaxed_ik::RelaxedIKService::Response,relaxed_ik_msg_RelaxedIKServiceRequest_common,relaxed_ik_msg_RelaxedIKServiceResponse_common>>();
  }
  std::shared_ptr<MATLABSvcClientInterface> 
          relaxed_ik_RelaxedIKService_service::generateSvcClientInterface(){
    return std::make_shared<ROSSvcClientImpl<relaxed_ik::RelaxedIKService,relaxed_ik::RelaxedIKService::Request,relaxed_ik::RelaxedIKService::Response,relaxed_ik_msg_RelaxedIKServiceRequest_common,relaxed_ik_msg_RelaxedIKServiceResponse_common>>();
  }
#include "ROSbagTemplates.hpp" 
  std::shared_ptr<MATLABRosbagWriterInterface> 
          relaxed_ik_RelaxedIKService_service::generateRosbagWriterInterface(ElementType type){
    std::shared_ptr<MATLABRosbagWriterInterface> ptr;
    if(type == eRequest){
        ptr = std::make_shared<ROSBagWriterImpl<relaxed_ik::RelaxedIKServiceRequest,relaxed_ik_msg_RelaxedIKServiceRequest_common>>();
    }else if(type == eResponse){
        ptr = std::make_shared<ROSBagWriterImpl<relaxed_ik::RelaxedIKServiceResponse,relaxed_ik_msg_RelaxedIKServiceResponse_common>>();
    }else{
        throw std::invalid_argument("Wrong input, Expected 'Request' or 'Response'");
    }
    return ptr;
  }
#include "register_macro.hpp"
// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
CLASS_LOADER_REGISTER_CLASS(relaxed_ik_msg_RelaxedIKServiceRequest_common, MATLABROSMsgInterface<relaxed_ik::RelaxedIKServiceRequest>)
CLASS_LOADER_REGISTER_CLASS(relaxed_ik_msg_RelaxedIKServiceResponse_common, MATLABROSMsgInterface<relaxed_ik::RelaxedIKServiceResponse>)
CLASS_LOADER_REGISTER_CLASS(relaxed_ik_RelaxedIKService_service, ROSMsgElementInterfaceFactory)
#ifdef _MSC_VER
#pragma warning(pop)
#else
#pragma GCC diagnostic pop
#endif //_MSC_VER
//gen-1
