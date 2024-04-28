// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CartesianControlServerROS2.hpp"

#include <string>

#include <yarp/os/LogStream.h>
#include <yarp/os/Property.h>
#include <yarp/os/Value.h>

#include "LogComponent.hpp"

using namespace roboticslab;

constexpr auto DEFAULT_NODE_NAME = "cartesian_control_server_ros2";
constexpr auto DEFAULT_MS = 20;

// ------------------- DeviceDriver Related ------------------------------------

bool CartesianControlServerROS2::open(yarp::os::Searchable & config)
{
    yarp::os::Value * name;

    if (config.check("subdevice", name))
    {
        yCInfo(CCS) << "Subdevice" << name->toString();

        if (name->isString())
        {
            // maybe user isn't doing nested configuration
            yarp::os::Property p;
            p.fromString(config.toString());
            p.put("device", name->toString());
            cartesianControlDevice.open(p);
        }
        else
        {
            cartesianControlDevice.open(*name);
        }

        if (!cartesianControlDevice.isValid())
        {
            yCError(CCS) << "Cannot make" << name->toString();
        }
    }
    else
    {
        yCError(CCS) << "Subdevice option not set in CartesianControlServerROS2";
        return false;
    }

    if (!cartesianControlDevice.isValid())
    {
        yCError(CCS) << "Cartesian control device not valid";
        return false;
    }

    if (!cartesianControlDevice.view(m_iCartesianControl))
    {
        yCError(CCS) << "Problems acquiring ICartesianControl interface";
        return false;
    }

    int periodInMs = config.check("period", yarp::os::Value(DEFAULT_MS), "FK stream period (milliseconds)").asInt32();

    if (periodInMs > 0)
    {
        yarp::os::PeriodicThread::setPeriod(periodInMs * 0.001);
    }
    else
    {
        yCWarning(CCS) << "Invalid period, using default";
        yarp::os::PeriodicThread::setPeriod(DEFAULT_MS * 0.001);
    }

    auto nodeName = config.check("node", yarp::os::Value(DEFAULT_NODE_NAME), "ROS node name").asString();
    auto prefix = "/" + nodeName;

    if (!rclcpp::ok())
    {
        rclcpp::init(0, nullptr);
    }

    // ROS2 initialization
    m_node = std::make_shared<rclcpp::Node>(nodeName);
    m_publisher = m_node->create_publisher<geometry_msgs::msg::PoseStamped>(prefix + "/state/pose", 10);

    m_poseSubscription = m_node->create_subscription<geometry_msgs::msg::Pose>(prefix + "/command/pose", 10,
                                                                               std::bind(&CartesianControlServerROS2::poseTopic_callback,
                                                                               this, std::placeholders::_1));
    if(!m_poseSubscription){
            yCError(CCS) << "Could not initialize the Pose msg subscription";
            return false;
        }     
    
    m_twistSubscription = m_node->create_subscription<geometry_msgs::msg::Twist>(prefix + "/command/twist", 10,
                                                                                 std::bind(&CartesianControlServerROS2::twistTopic_callback,
                                                                                 this, std::placeholders::_1));
    if(!m_twistSubscription){
            yCError(CCS) << "Could not initialize the Twist msg subscription";
            return false;
        }
    
    m_wrenchSubscription = m_node->create_subscription<geometry_msgs::msg::Wrench>(prefix + "/command/wrench", 10,
                                                                                   std::bind(&CartesianControlServerROS2::wrenchTopic_callback,
                                                                                   this, std::placeholders::_1));
    if(!m_wrenchSubscription){
            yCError(CCS) << "Could not initialize the Wrench msg subscription";
            return false;
        }

    m_spinner = new Spinner(m_node);

    if (!m_spinner)
    {
        yCError(CCS) << "Failed to create spinner";
        return false;
    }

    return m_spinner->start() && yarp::os::PeriodicThread::start();
}

// -----------------------------------------------------------------------------

bool CartesianControlServerROS2::close()
{
     if (m_spinner && m_spinner->isRunning())
    {
        m_spinner->stop();
    }
 
    return cartesianControlDevice.close();
}

// -----------------------------------------------------------------------------
/********************************SPINNER CLASS*********************************/
// -----------------------------------------------------------------------------

Spinner::Spinner(std::shared_ptr<rclcpp::Node> input_node)
    : m_node(input_node)
{}

// -----------------------------------------------------------------------------

Spinner::~Spinner()
{
    if (m_spun)
    {
        rclcpp::shutdown();
        m_spun = false;
    }
}

// -----------------------------------------------------------------------------

void Spinner::run()
{
    if (!m_spun)
    {
        m_spun = true;
        rclcpp::spin(m_node);
    }
}

// -----------------------------------------------------------------------------
