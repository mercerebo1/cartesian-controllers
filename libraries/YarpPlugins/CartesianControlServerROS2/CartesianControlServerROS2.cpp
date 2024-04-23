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
