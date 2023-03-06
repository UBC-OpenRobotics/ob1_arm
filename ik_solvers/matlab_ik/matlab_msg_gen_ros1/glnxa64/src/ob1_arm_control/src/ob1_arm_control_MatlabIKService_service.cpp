// Copyright 2019-2021 The MathWorks, Inc.
// Common copy functions for ob1_arm_control/MatlabIKServiceRequest
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
#include "ob1_arm_control/MatlabIKService.h"
#include "visibility_control.h"
#include "ROSPubSubTemplates.hpp"
#include "ROSServiceTemplates.hpp"
class OB1_ARM_CONTROL_EXPORT ob1_arm_control_msg_MatlabIKServiceRequest_common : public MATLABROSMsgInterface<ob1_arm_control::MatlabIKService::Request> {
  public:
    virtual ~ob1_arm_control_msg_MatlabIKServiceRequest_common(){}
    virtual void copy_from_struct(ob1_arm_control::MatlabIKService::Request* msg, const matlab::data::Struct& arr, MultiLibLoader loader); 
    //----------------------------------------------------------------------------
    virtual MDArray_T get_arr(MDFactory_T& factory, const ob1_arm_control::MatlabIKService::Request* msg, MultiLibLoader loader, size_t size = 1);
};
  void ob1_arm_control_msg_MatlabIKServiceRequest_common::copy_from_struct(ob1_arm_control::MatlabIKService::Request* msg, const matlab::data::Struct& arr,
               MultiLibLoader loader) {
    try {
        //pose_target
        const matlab::data::TypedArray<double> pose_target_arr = arr["PoseTarget"];
        size_t nelem = pose_target_arr.getNumberOfElements();
        	msg->pose_target.resize(nelem);
        	std::copy(pose_target_arr.begin(), pose_target_arr.begin()+nelem, msg->pose_target.begin());
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'PoseTarget' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'PoseTarget' is wrong type; expected a double.");
    }
    try {
        //tolerance
        const matlab::data::TypedArray<double> tolerance_arr = arr["Tolerance"];
        size_t nelem = tolerance_arr.getNumberOfElements();
        	msg->tolerance.resize(nelem);
        	std::copy(tolerance_arr.begin(), tolerance_arr.begin()+nelem, msg->tolerance.begin());
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'Tolerance' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'Tolerance' is wrong type; expected a double.");
    }
  }
  //----------------------------------------------------------------------------
  MDArray_T ob1_arm_control_msg_MatlabIKServiceRequest_common::get_arr(MDFactory_T& factory, const ob1_arm_control::MatlabIKService::Request* msg,
       MultiLibLoader loader, size_t size) {
    auto outArray = factory.createStructArray({size,1},{"MessageType","PoseTarget","Tolerance"});
    for(size_t ctr = 0; ctr < size; ctr++){
    outArray[ctr]["MessageType"] = factory.createCharArray("ob1_arm_control/MatlabIKServiceRequest");
    // pose_target
    auto currentElement_pose_target = (msg + ctr)->pose_target;
    outArray[ctr]["PoseTarget"] = factory.createArray<ob1_arm_control::MatlabIKService::Request::_pose_target_type::const_iterator, double>({currentElement_pose_target.size(),1}, currentElement_pose_target.begin(), currentElement_pose_target.end());
    // tolerance
    auto currentElement_tolerance = (msg + ctr)->tolerance;
    outArray[ctr]["Tolerance"] = factory.createArray<ob1_arm_control::MatlabIKService::Request::_tolerance_type::const_iterator, double>({currentElement_tolerance.size(),1}, currentElement_tolerance.begin(), currentElement_tolerance.end());
    }
    return std::move(outArray);
  }
class OB1_ARM_CONTROL_EXPORT ob1_arm_control_msg_MatlabIKServiceResponse_common : public MATLABROSMsgInterface<ob1_arm_control::MatlabIKService::Response> {
  public:
    virtual ~ob1_arm_control_msg_MatlabIKServiceResponse_common(){}
    virtual void copy_from_struct(ob1_arm_control::MatlabIKService::Response* msg, const matlab::data::Struct& arr, MultiLibLoader loader); 
    //----------------------------------------------------------------------------
    virtual MDArray_T get_arr(MDFactory_T& factory, const ob1_arm_control::MatlabIKService::Response* msg, MultiLibLoader loader, size_t size = 1);
};
  void ob1_arm_control_msg_MatlabIKServiceResponse_common::copy_from_struct(ob1_arm_control::MatlabIKService::Response* msg, const matlab::data::Struct& arr,
               MultiLibLoader loader) {
    try {
        //result
        const matlab::data::CharArray result_arr = arr["Result"];
        msg->result = result_arr.toAscii();
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'Result' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'Result' is wrong type; expected a string.");
    }
    try {
        //error
        const matlab::data::TypedArray<double> error_arr = arr["Error"];
        msg->error = error_arr[0];
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'Error' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'Error' is wrong type; expected a double.");
    }
    try {
        //joints
        const matlab::data::TypedArray<double> joints_arr = arr["Joints"];
        size_t nelem = joints_arr.getNumberOfElements();
        	msg->joints.resize(nelem);
        	std::copy(joints_arr.begin(), joints_arr.begin()+nelem, msg->joints.begin());
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'Joints' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'Joints' is wrong type; expected a double.");
    }
  }
  //----------------------------------------------------------------------------
  MDArray_T ob1_arm_control_msg_MatlabIKServiceResponse_common::get_arr(MDFactory_T& factory, const ob1_arm_control::MatlabIKService::Response* msg,
       MultiLibLoader loader, size_t size) {
    auto outArray = factory.createStructArray({size,1},{"MessageType","Result","Error","Joints"});
    for(size_t ctr = 0; ctr < size; ctr++){
    outArray[ctr]["MessageType"] = factory.createCharArray("ob1_arm_control/MatlabIKServiceResponse");
    // result
    auto currentElement_result = (msg + ctr)->result;
    outArray[ctr]["Result"] = factory.createCharArray(currentElement_result);
    // error
    auto currentElement_error = (msg + ctr)->error;
    outArray[ctr]["Error"] = factory.createScalar(currentElement_error);
    // joints
    auto currentElement_joints = (msg + ctr)->joints;
    outArray[ctr]["Joints"] = factory.createArray<ob1_arm_control::MatlabIKService::Response::_joints_type::const_iterator, double>({currentElement_joints.size(),1}, currentElement_joints.begin(), currentElement_joints.end());
    }
    return std::move(outArray);
  } 
class OB1_ARM_CONTROL_EXPORT ob1_arm_control_MatlabIKService_service : public ROSMsgElementInterfaceFactory {
  public:
    virtual ~ob1_arm_control_MatlabIKService_service(){}
    virtual std::shared_ptr<MATLABPublisherInterface> generatePublisherInterface(ElementType type);
    virtual std::shared_ptr<MATLABSubscriberInterface> generateSubscriberInterface(ElementType type);
    virtual std::shared_ptr<MATLABRosbagWriterInterface> generateRosbagWriterInterface(ElementType type);
    virtual std::shared_ptr<MATLABSvcServerInterface> generateSvcServerInterface();
    virtual std::shared_ptr<MATLABSvcClientInterface> generateSvcClientInterface();
};  
  std::shared_ptr<MATLABPublisherInterface> 
          ob1_arm_control_MatlabIKService_service::generatePublisherInterface(ElementType type){
    std::shared_ptr<MATLABPublisherInterface> ptr;
    if(type == eRequest){
        ptr = std::make_shared<ROSPublisherImpl<ob1_arm_control::MatlabIKService::Request,ob1_arm_control_msg_MatlabIKServiceRequest_common>>();
    }else if(type == eResponse){
        ptr = std::make_shared<ROSPublisherImpl<ob1_arm_control::MatlabIKService::Response,ob1_arm_control_msg_MatlabIKServiceResponse_common>>();
    }else{
        throw std::invalid_argument("Wrong input, Expected 'Request' or 'Response'");
    }
    return ptr;
  }
  std::shared_ptr<MATLABSubscriberInterface> 
          ob1_arm_control_MatlabIKService_service::generateSubscriberInterface(ElementType type){
    std::shared_ptr<MATLABSubscriberInterface> ptr;
    if(type == eRequest){
        ptr = std::make_shared<ROSSubscriberImpl<ob1_arm_control::MatlabIKService::Request,ob1_arm_control::MatlabIKService::Request::ConstPtr,ob1_arm_control_msg_MatlabIKServiceRequest_common>>();
    }else if(type == eResponse){
        ptr = std::make_shared<ROSSubscriberImpl<ob1_arm_control::MatlabIKService::Response,ob1_arm_control::MatlabIKService::Response::ConstPtr,ob1_arm_control_msg_MatlabIKServiceResponse_common>>();
    }else{
        throw std::invalid_argument("Wrong input, Expected 'Request' or 'Response'");
    }
    return ptr;
  }
  std::shared_ptr<MATLABSvcServerInterface> 
          ob1_arm_control_MatlabIKService_service::generateSvcServerInterface(){
    return std::make_shared<ROSSvcServerImpl<ob1_arm_control::MatlabIKService::Request,ob1_arm_control::MatlabIKService::Response,ob1_arm_control_msg_MatlabIKServiceRequest_common,ob1_arm_control_msg_MatlabIKServiceResponse_common>>();
  }
  std::shared_ptr<MATLABSvcClientInterface> 
          ob1_arm_control_MatlabIKService_service::generateSvcClientInterface(){
    return std::make_shared<ROSSvcClientImpl<ob1_arm_control::MatlabIKService,ob1_arm_control::MatlabIKService::Request,ob1_arm_control::MatlabIKService::Response,ob1_arm_control_msg_MatlabIKServiceRequest_common,ob1_arm_control_msg_MatlabIKServiceResponse_common>>();
  }
#include "ROSbagTemplates.hpp" 
  std::shared_ptr<MATLABRosbagWriterInterface> 
          ob1_arm_control_MatlabIKService_service::generateRosbagWriterInterface(ElementType type){
    std::shared_ptr<MATLABRosbagWriterInterface> ptr;
    if(type == eRequest){
        ptr = std::make_shared<ROSBagWriterImpl<ob1_arm_control::MatlabIKServiceRequest,ob1_arm_control_msg_MatlabIKServiceRequest_common>>();
    }else if(type == eResponse){
        ptr = std::make_shared<ROSBagWriterImpl<ob1_arm_control::MatlabIKServiceResponse,ob1_arm_control_msg_MatlabIKServiceResponse_common>>();
    }else{
        throw std::invalid_argument("Wrong input, Expected 'Request' or 'Response'");
    }
    return ptr;
  }
#include "register_macro.hpp"
// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
CLASS_LOADER_REGISTER_CLASS(ob1_arm_control_msg_MatlabIKServiceRequest_common, MATLABROSMsgInterface<ob1_arm_control::MatlabIKServiceRequest>)
CLASS_LOADER_REGISTER_CLASS(ob1_arm_control_msg_MatlabIKServiceResponse_common, MATLABROSMsgInterface<ob1_arm_control::MatlabIKServiceResponse>)
CLASS_LOADER_REGISTER_CLASS(ob1_arm_control_MatlabIKService_service, ROSMsgElementInterfaceFactory)
#ifdef _MSC_VER
#pragma warning(pop)
#else
#pragma GCC diagnostic pop
#endif //_MSC_VER
//gen-1
