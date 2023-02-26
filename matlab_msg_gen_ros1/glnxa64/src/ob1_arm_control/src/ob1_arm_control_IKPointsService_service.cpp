// Copyright 2019-2021 The MathWorks, Inc.
// Common copy functions for ob1_arm_control/IKPointsServiceRequest
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
#include "ob1_arm_control/IKPointsService.h"
#include "visibility_control.h"
#include "ROSPubSubTemplates.hpp"
#include "ROSServiceTemplates.hpp"
class OB1_ARM_CONTROL_EXPORT ob1_arm_control_msg_IKPointsServiceRequest_common : public MATLABROSMsgInterface<ob1_arm_control::IKPointsService::Request> {
  public:
    virtual ~ob1_arm_control_msg_IKPointsServiceRequest_common(){}
    virtual void copy_from_struct(ob1_arm_control::IKPointsService::Request* msg, const matlab::data::Struct& arr, MultiLibLoader loader); 
    //----------------------------------------------------------------------------
    virtual MDArray_T get_arr(MDFactory_T& factory, const ob1_arm_control::IKPointsService::Request* msg, MultiLibLoader loader, size_t size = 1);
};
  void ob1_arm_control_msg_IKPointsServiceRequest_common::copy_from_struct(ob1_arm_control::IKPointsService::Request* msg, const matlab::data::Struct& arr,
               MultiLibLoader loader) {
    try {
        //request
        const matlab::data::CharArray request_arr = arr["Request"];
        msg->request = request_arr.toAscii();
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'Request' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'Request' is wrong type; expected a string.");
    }
    try {
        //pose
        const matlab::data::StructArray pose_arr = arr["Pose"];
        auto msgClassPtr_pose = getCommonObject<geometry_msgs::Pose>("geometry_msgs_msg_Pose_common",loader);
        msgClassPtr_pose->copy_from_struct(&msg->pose,pose_arr[0],loader);
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'Pose' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'Pose' is wrong type; expected a struct.");
    }
    try {
        //num_pts
        const matlab::data::TypedArray<uint32_t> num_pts_arr = arr["NumPts"];
        msg->num_pts = num_pts_arr[0];
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'NumPts' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'NumPts' is wrong type; expected a uint32.");
    }
    try {
        //tolerance
        const matlab::data::TypedArray<float> tolerance_arr = arr["Tolerance"];
        msg->tolerance = tolerance_arr[0];
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'Tolerance' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'Tolerance' is wrong type; expected a single.");
    }
    try {
        //distance
        const matlab::data::TypedArray<float> distance_arr = arr["Distance"];
        msg->distance = distance_arr[0];
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'Distance' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'Distance' is wrong type; expected a single.");
    }
  }
  //----------------------------------------------------------------------------
  MDArray_T ob1_arm_control_msg_IKPointsServiceRequest_common::get_arr(MDFactory_T& factory, const ob1_arm_control::IKPointsService::Request* msg,
       MultiLibLoader loader, size_t size) {
    auto outArray = factory.createStructArray({size,1},{"MessageType","Request","Pose","NumPts","Tolerance","Distance"});
    for(size_t ctr = 0; ctr < size; ctr++){
    outArray[ctr]["MessageType"] = factory.createCharArray("ob1_arm_control/IKPointsServiceRequest");
    // request
    auto currentElement_request = (msg + ctr)->request;
    outArray[ctr]["Request"] = factory.createCharArray(currentElement_request);
    // pose
    auto currentElement_pose = (msg + ctr)->pose;
    auto msgClassPtr_pose = getCommonObject<geometry_msgs::Pose>("geometry_msgs_msg_Pose_common",loader);
    outArray[ctr]["Pose"] = msgClassPtr_pose->get_arr(factory, &currentElement_pose, loader);
    // num_pts
    auto currentElement_num_pts = (msg + ctr)->num_pts;
    outArray[ctr]["NumPts"] = factory.createScalar(currentElement_num_pts);
    // tolerance
    auto currentElement_tolerance = (msg + ctr)->tolerance;
    outArray[ctr]["Tolerance"] = factory.createScalar(currentElement_tolerance);
    // distance
    auto currentElement_distance = (msg + ctr)->distance;
    outArray[ctr]["Distance"] = factory.createScalar(currentElement_distance);
    }
    return std::move(outArray);
  }
class OB1_ARM_CONTROL_EXPORT ob1_arm_control_msg_IKPointsServiceResponse_common : public MATLABROSMsgInterface<ob1_arm_control::IKPointsService::Response> {
  public:
    virtual ~ob1_arm_control_msg_IKPointsServiceResponse_common(){}
    virtual void copy_from_struct(ob1_arm_control::IKPointsService::Response* msg, const matlab::data::Struct& arr, MultiLibLoader loader); 
    //----------------------------------------------------------------------------
    virtual MDArray_T get_arr(MDFactory_T& factory, const ob1_arm_control::IKPointsService::Response* msg, MultiLibLoader loader, size_t size = 1);
};
  void ob1_arm_control_msg_IKPointsServiceResponse_common::copy_from_struct(ob1_arm_control::IKPointsService::Response* msg, const matlab::data::Struct& arr,
               MultiLibLoader loader) {
    try {
        //pose_targets
        const matlab::data::StructArray pose_targets_arr = arr["PoseTargets"];
        for (auto _pose_targetsarr : pose_targets_arr) {
        	geometry_msgs::Pose _val;
        auto msgClassPtr_pose_targets = getCommonObject<geometry_msgs::Pose>("geometry_msgs_msg_Pose_common",loader);
        msgClassPtr_pose_targets->copy_from_struct(&_val,_pose_targetsarr,loader);
        	msg->pose_targets.push_back(_val);
        }
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'PoseTargets' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'PoseTargets' is wrong type; expected a struct.");
    }
    try {
        //joint_targets
        const matlab::data::StructArray joint_targets_arr = arr["JointTargets"];
        for (auto _joint_targetsarr : joint_targets_arr) {
        	ob1_arm_control::JointTarget _val;
        auto msgClassPtr_joint_targets = getCommonObject<ob1_arm_control::JointTarget>("ob1_arm_control_msg_JointTarget_common",loader);
        msgClassPtr_joint_targets->copy_from_struct(&_val,_joint_targetsarr,loader);
        	msg->joint_targets.push_back(_val);
        }
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'JointTargets' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'JointTargets' is wrong type; expected a struct.");
    }
    try {
        //condition
        const matlab::data::TypedArray<bool> condition_arr = arr["Condition"];
        msg->condition = condition_arr[0];
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'Condition' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'Condition' is wrong type; expected a logical.");
    }
  }
  //----------------------------------------------------------------------------
  MDArray_T ob1_arm_control_msg_IKPointsServiceResponse_common::get_arr(MDFactory_T& factory, const ob1_arm_control::IKPointsService::Response* msg,
       MultiLibLoader loader, size_t size) {
    auto outArray = factory.createStructArray({size,1},{"MessageType","PoseTargets","JointTargets","Condition"});
    for(size_t ctr = 0; ctr < size; ctr++){
    outArray[ctr]["MessageType"] = factory.createCharArray("ob1_arm_control/IKPointsServiceResponse");
    // pose_targets
    auto currentElement_pose_targets = (msg + ctr)->pose_targets;
    auto msgClassPtr_pose_targets = getCommonObject<geometry_msgs::Pose>("geometry_msgs_msg_Pose_common",loader);
    outArray[ctr]["PoseTargets"] = msgClassPtr_pose_targets->get_arr(factory,&currentElement_pose_targets[0],loader,currentElement_pose_targets.size());
    // joint_targets
    auto currentElement_joint_targets = (msg + ctr)->joint_targets;
    auto msgClassPtr_joint_targets = getCommonObject<ob1_arm_control::JointTarget>("ob1_arm_control_msg_JointTarget_common",loader);
    outArray[ctr]["JointTargets"] = msgClassPtr_joint_targets->get_arr(factory,&currentElement_joint_targets[0],loader,currentElement_joint_targets.size());
    // condition
    auto currentElement_condition = (msg + ctr)->condition;
    outArray[ctr]["Condition"] = factory.createScalar(static_cast<bool>(currentElement_condition));
    }
    return std::move(outArray);
  } 
class OB1_ARM_CONTROL_EXPORT ob1_arm_control_IKPointsService_service : public ROSMsgElementInterfaceFactory {
  public:
    virtual ~ob1_arm_control_IKPointsService_service(){}
    virtual std::shared_ptr<MATLABPublisherInterface> generatePublisherInterface(ElementType type);
    virtual std::shared_ptr<MATLABSubscriberInterface> generateSubscriberInterface(ElementType type);
    virtual std::shared_ptr<MATLABRosbagWriterInterface> generateRosbagWriterInterface(ElementType type);
    virtual std::shared_ptr<MATLABSvcServerInterface> generateSvcServerInterface();
    virtual std::shared_ptr<MATLABSvcClientInterface> generateSvcClientInterface();
};  
  std::shared_ptr<MATLABPublisherInterface> 
          ob1_arm_control_IKPointsService_service::generatePublisherInterface(ElementType type){
    std::shared_ptr<MATLABPublisherInterface> ptr;
    if(type == eRequest){
        ptr = std::make_shared<ROSPublisherImpl<ob1_arm_control::IKPointsService::Request,ob1_arm_control_msg_IKPointsServiceRequest_common>>();
    }else if(type == eResponse){
        ptr = std::make_shared<ROSPublisherImpl<ob1_arm_control::IKPointsService::Response,ob1_arm_control_msg_IKPointsServiceResponse_common>>();
    }else{
        throw std::invalid_argument("Wrong input, Expected 'Request' or 'Response'");
    }
    return ptr;
  }
  std::shared_ptr<MATLABSubscriberInterface> 
          ob1_arm_control_IKPointsService_service::generateSubscriberInterface(ElementType type){
    std::shared_ptr<MATLABSubscriberInterface> ptr;
    if(type == eRequest){
        ptr = std::make_shared<ROSSubscriberImpl<ob1_arm_control::IKPointsService::Request,ob1_arm_control::IKPointsService::Request::ConstPtr,ob1_arm_control_msg_IKPointsServiceRequest_common>>();
    }else if(type == eResponse){
        ptr = std::make_shared<ROSSubscriberImpl<ob1_arm_control::IKPointsService::Response,ob1_arm_control::IKPointsService::Response::ConstPtr,ob1_arm_control_msg_IKPointsServiceResponse_common>>();
    }else{
        throw std::invalid_argument("Wrong input, Expected 'Request' or 'Response'");
    }
    return ptr;
  }
  std::shared_ptr<MATLABSvcServerInterface> 
          ob1_arm_control_IKPointsService_service::generateSvcServerInterface(){
    return std::make_shared<ROSSvcServerImpl<ob1_arm_control::IKPointsService::Request,ob1_arm_control::IKPointsService::Response,ob1_arm_control_msg_IKPointsServiceRequest_common,ob1_arm_control_msg_IKPointsServiceResponse_common>>();
  }
  std::shared_ptr<MATLABSvcClientInterface> 
          ob1_arm_control_IKPointsService_service::generateSvcClientInterface(){
    return std::make_shared<ROSSvcClientImpl<ob1_arm_control::IKPointsService,ob1_arm_control::IKPointsService::Request,ob1_arm_control::IKPointsService::Response,ob1_arm_control_msg_IKPointsServiceRequest_common,ob1_arm_control_msg_IKPointsServiceResponse_common>>();
  }
#include "ROSbagTemplates.hpp" 
  std::shared_ptr<MATLABRosbagWriterInterface> 
          ob1_arm_control_IKPointsService_service::generateRosbagWriterInterface(ElementType type){
    std::shared_ptr<MATLABRosbagWriterInterface> ptr;
    if(type == eRequest){
        ptr = std::make_shared<ROSBagWriterImpl<ob1_arm_control::IKPointsServiceRequest,ob1_arm_control_msg_IKPointsServiceRequest_common>>();
    }else if(type == eResponse){
        ptr = std::make_shared<ROSBagWriterImpl<ob1_arm_control::IKPointsServiceResponse,ob1_arm_control_msg_IKPointsServiceResponse_common>>();
    }else{
        throw std::invalid_argument("Wrong input, Expected 'Request' or 'Response'");
    }
    return ptr;
  }
#include "register_macro.hpp"
// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
CLASS_LOADER_REGISTER_CLASS(ob1_arm_control_msg_IKPointsServiceRequest_common, MATLABROSMsgInterface<ob1_arm_control::IKPointsServiceRequest>)
CLASS_LOADER_REGISTER_CLASS(ob1_arm_control_msg_IKPointsServiceResponse_common, MATLABROSMsgInterface<ob1_arm_control::IKPointsServiceResponse>)
CLASS_LOADER_REGISTER_CLASS(ob1_arm_control_IKPointsService_service, ROSMsgElementInterfaceFactory)
#ifdef _MSC_VER
#pragma warning(pop)
#else
#pragma GCC diagnostic pop
#endif //_MSC_VER
//gen-1
