// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CartesianControlServerROS2.hpp"
#include "LogComponent.hpp"

#include <cmath> 

#include <memory>
#include <vector>
#include <mutex>

#include <kdl/chain.hpp> 
#include <kdl/chainiksolvervel_pinv.hpp>

#include <yarp/os/LogStream.h>
#include <yarp/os/Property.h>
#include <yarp/os/Value.h>


using namespace roboticslab;


void CartesianControlServerROS2::poseTopic_callback(const geometry_msgs::msg::Pose::SharedPtr msg)
{
    //std::lock_guard <std::mutex> lg(m_mtx);

    const auto ori = KDL::Rotation::Quaternion(msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);
    const auto rot = ori.GetRot();

    std::vector<double> v {
        msg->position.x,
        msg->position.y,
        msg->position.z,
        rot.x(),
        rot.y(),
        rot.z()
    };

    yCInfo(CCS) << "Received pose: [ " << v[0] << v[1] << v[2] << v[3] << v[4] << v[5] << " ]";
    RCLCPP_INFO(m_node->get_logger(),"Received pose: [ %f %f %f %f %f %f ]",v[0],v[1],v[2],v[3],v[4],v[5] );

    m_iCartesianControl->movi(v);
}

// -----------------------------------------------------------------------------

void CartesianControlServerROS2::twistTopic_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    //std::lock_guard <std::mutex> lg(m_mtx);

    std::vector<double> v {
        msg->linear.x,
        msg->linear.y,
        msg->linear.z,
        msg->angular.x,
        msg->angular.y,
        msg->angular.z
    };


    yCInfo(CCS) << "Received twist: [ " << v[0] << v[1] << v[2] << v[3] << v[4] << v[5] << " ]";
    RCLCPP_INFO(m_node->get_logger(),"Received twist: [ %f %f %f %f %f %f ]",v[0],v[1],v[2],v[3],v[4],v[5] );

    m_iCartesianControl->twist(v);
}

// -----------------------------------------------------------------------------

void CartesianControlServerROS2::wrenchTopic_callback(const geometry_msgs::msg::Wrench::SharedPtr msg)
{
    //std::lock_guard <std::mutex> lg(m_mtx);

    std::vector<double> v {
        msg->force.x,
        msg->force.y,
        msg->force.z,
        msg->torque.x,
        msg->torque.y,
        msg->torque.z
    };
    
    yCInfo(CCS) << "Received wrench: [ " << v[0] << v[1] << v[2] << v[3] << v[4] << v[5] << " ]";
    RCLCPP_INFO(m_node->get_logger(),"Received wrench: [ %f %f %f %f %f %f ]",v[0],v[1],v[2],v[3],v[4],v[5] );

    m_iCartesianControl->wrench(v);
}

// -----------------------------------------------------------------------------

 
    //----- # Service to get the currently set control modes for a set of joints -----
    // # names: it can contain the names of the joints from which the control mode has to be retrieved. If left empty, the control modes of all the available joints will be returned
    // string[] names
    // ---
    // # modes: it will cotain the control modes for the joints specified in "names" or for all them if "names" is empty
    // # response: a brief string used to signal the state of the result of the request
    // # opt_descr: An optional human readable description of the result of the request
    // string[] modes
    // string response "NOT_SPECIFIED"
    // string opt_descr
    
// void ControlBoard_nws_ros2::getControlModesCallback(const std::shared_ptr<rmw_request_id_t> request_header,
//                                                     const std::shared_ptr<yarp_control_msgs::srv::GetControlModes::Request> request,
//                                                     std::shared_ptr<yarp_control_msgs::srv::GetControlModes::Response> response){
//     std::lock_guard <std::mutex> lg(m_cmdMutex);

//     bool noJoints = request->names.size() == 0;

//     if(!request){
//         yCError(CCS) << "Invalid request";
//         RCLCPP_ERROR(m_node->get_logger(),"Invalid request");

//         response->response = "INVALID";

//         return;
//     }

//     if(!noJoints){
//         if(!namesCheck(request->names)){
//             response->response = "NAMES_ERROR";

//             return;
//         }
//     }

//     size_t forLimit = noJoints ? m_subdevice_joints : request->names.size();
//     int *tempMode = new int[1];
//     std::vector<std::string> modesToSend;

//     for (size_t i=0; i<forLimit; i++){

//         if(!m_iControlMode->getControlMode(noJoints ? i : m_quickJointRef[request->names[i]],tempMode)){
//             yCError(CCS) << "Error while retrieving the control mode for joint"<<request->names[i];
//             RCLCPP_ERROR(m_node->get_logger(),"Error while retrieving the control mode for joint %s",request->names[i].c_str());
//             response->response = "RETRIEVE_ERROR";

//             delete tempMode;
//             return;
//         }
//         modesToSend.push_back(fromCtrlModeToString.at(*tempMode));
//     }
//     response->modes = modesToSend;
//     response->response = "OK";

//     delete tempMode;
// }

// -----------------------------------------------------------------------------
