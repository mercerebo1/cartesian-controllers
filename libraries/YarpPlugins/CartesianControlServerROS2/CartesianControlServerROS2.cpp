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

// ------------------- Subscription callbacks ------------------------------------

void CartesianControlServerROS2::poseTopic_callback(const geometry_msgs::msg::Pose::SharedPtr msg)
{
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

    m_iCartesianControl->setParameter(VOCAB_CC_CONFIG_STREAMING_CMD, VOCAB_CC_MOVI);

    m_node->set_parameter(rclcpp::Parameter("preset_streaming_cmd", "movi")); // Setting corresponding streaming parameter in ROS2 node
    
    yCInfo(CCS) << "Received pose: [ " << v[0] << v[1] << v[2] << v[3] << v[4] << v[5] << " ]";
    
    m_iCartesianControl->movi(v);
}

// -----------------------------------------------------------------------------

void CartesianControlServerROS2::twistTopic_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    std::vector<double> v {
        msg->linear.x,
        msg->linear.y,
        msg->linear.z,
        msg->angular.x,
        msg->angular.y,
        msg->angular.z
    };

    m_iCartesianControl->setParameter(VOCAB_CC_CONFIG_STREAMING_CMD, VOCAB_CC_TWIST);

    m_node->set_parameter(rclcpp::Parameter("preset_streaming_cmd", "twist"));

    yCInfo(CCS) << "Received twist: [ " << v[0] << v[1] << v[2] << v[3] << v[4] << v[5] << " ]";
    
    m_iCartesianControl->twist(v);
}

// -----------------------------------------------------------------------------

void CartesianControlServerROS2::wrenchTopic_callback(const geometry_msgs::msg::Wrench::SharedPtr msg)
{
    std::vector<double> v {
        msg->force.x,
        msg->force.y,
        msg->force.z,
        msg->torque.x,
        msg->torque.y,
        msg->torque.z
    };
    
    m_iCartesianControl->setParameter(VOCAB_CC_CONFIG_STREAMING_CMD, VOCAB_CC_WRENCH);

    m_node->set_parameter(rclcpp::Parameter("preset_streaming_cmd", "wrench"));

    yCInfo(CCS) << "Received wrench: [ " << v[0] << v[1] << v[2] << v[3] << v[4] << v[5] << " ]";

    m_iCartesianControl->wrench(v);
}

// -----------------------------------------------------------------------------

rcl_interfaces::msg::SetParametersResult CartesianControlServerROS2::parameter_callback(
    const std::vector<rclcpp::Parameter> &parameters)
    {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        for (const auto &param: parameters)
        {
            if(param.get_name() == "preset_streaming_cmd")
            {
                preset_streaming_cmd = param.value_to_string();

                if (preset_streaming_cmd == "twist")
                {
                    yCInfo(CCS) << "Param for preset_streaming_cmd correctly stablished:" << preset_streaming_cmd.c_str();
                    m_iCartesianControl->setParameter(VOCAB_CC_CONFIG_STREAMING_CMD, VOCAB_CC_TWIST);
                }
                else if (preset_streaming_cmd == "movi")
                {
                    yCInfo(CCS) << "Param for preset_streaming_cmd correctly stablished:" << preset_streaming_cmd.c_str();
                    m_iCartesianControl->setParameter(VOCAB_CC_CONFIG_STREAMING_CMD, VOCAB_CC_MOVI);
                }
                else if (preset_streaming_cmd == "wrench")
                {
                    yCInfo(CCS) << "Param for preset_streaming_cmd correctly stablished:" << preset_streaming_cmd.c_str();
                    m_iCartesianControl->setParameter(VOCAB_CC_CONFIG_STREAMING_CMD, VOCAB_CC_WRENCH);
                }
                else
                {
                    result.successful = false;
                    result.reason = "Invalid parameter value. Only 'twist', 'movi', or 'wrench' are allowed.";
                    yCInfo(CCS) << "Invalid parameter value for preset_streaming_cmd.";
                }
            }
        }
        return result;
    }
// -----------------------------------------------------------------------------
