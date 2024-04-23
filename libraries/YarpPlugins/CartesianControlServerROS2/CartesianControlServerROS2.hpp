// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __CARTESIAN_CONTROL_SERVER_HPP__
#define __CARTESIAN_CONTROL_SERVER_HPP__

#include <yarp/os/PeriodicThread.h>

#include <yarp/dev/Drivers.h>
#include <yarp/dev/PolyDriver.h>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/wrench.hpp>

#include <kdl/frames.hpp>

#include <ICartesianControl.h>

//Custom ros2 interfaces
// #include <yarp_control_msgs/srv/get_control_modes.hpp>
// #include <yarp_control_msgs/srv/get_position.hpp>
// #include <yarp_control_msgs/srv/get_velocity.hpp>
// #include <yarp_control_msgs/srv/set_control_modes.hpp>
// #include <yarp_control_msgs/srv/get_available_control_modes.hpp>
// #include <yarp_control_msgs/srv/get_joints_names.hpp>
// #include <yarp_control_msgs/msg/position.hpp>
// #include <yarp_control_msgs/msg/velocity.hpp>
// #include <yarp_control_msgs/msg/position_direct.hpp>

namespace roboticslab
{

class CartesianControlServerROS2 : public yarp::dev::DeviceDriver,
                                   public yarp::os::PeriodicThread
{
public:
    CartesianControlServerROS2() : yarp::os::PeriodicThread(1.0)
    {}

    // yarp::dev::DeviceDriver declarations -- Implementation in DeviceDriverImpl.cpp 
    bool open(yarp::os::Searchable & config) override;
    bool close() override;

    // yarp::os::PeriodicThread declarations --- Implementation in PeriodicThread.cpp 
    void run() override;

    // yarp::dev::IWrapper
    // bool attach(yarp::dev::PolyDriver* poly) override;
    // bool detach() override;

private:

    mutable std::mutex m_mtx;

    // -- Devices --
    yarp::dev::PolyDriver cartesianControlDevice;
    ICartesianControl * m_iCartesianControl;
    // yarp::dev::IAxisInfo*        m_iAxisInfo{nullptr};
    // yarp::dev::IEncodersTimed*   m_iEncodersTimed{nullptr};
    // yarp::dev::ITorqueControl*   m_iTorqueControl{nullptr};
    // yarp::dev::IPositionDirect*  m_iPositionDirect{nullptr};
    // yarp::dev::IVelocityControl* m_iVelocityControl{nullptr};
    // yarp::dev::IControlMode*     m_iControlMode{nullptr};
    // yarp::dev::IPositionControl* m_iPositionControl{nullptr};

    // -- ROS2 attributes -- 
    rclcpp::Node::SharedPtr m_node;
    // Publisher
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr m_publisher;
    // Subscriptions
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr m_poseSubscription;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr m_twistSubscription;
    rclcpp::Subscription<geometry_msgs::msg::Wrench>::SharedPtr m_wrenchSubscription;

    // Services
    // rclcpp::Service<yarp_control_msgs::srv::GetJointsNames>::SharedPtr           m_getJointsNamesSrv;
    // rclcpp::Service<yarp_control_msgs::srv::GetControlModes>::SharedPtr          m_getControlModesSrv;
    // rclcpp::Service<yarp_control_msgs::srv::GetPosition>::SharedPtr              m_getPositionSrv;
    // rclcpp::Service<yarp_control_msgs::srv::GetVelocity>::SharedPtr              m_getVelocitySrv;
    // rclcpp::Service<yarp_control_msgs::srv::SetControlModes>::SharedPtr          m_setControlModesSrv;
    // rclcpp::Service<yarp_control_msgs::srv::GetAvailableControlModes>::SharedPtr m_getAvailableModesSrv;


    // -- Subscription callbacks - Topics --
    void poseTopic_callback(const geometry_msgs::msg::Pose::SharedPtr msg);
    void twistTopic_callback(const geometry_msgs::msg::Twist::SharedPtr msg);
    void wrenchTopic_callback(const geometry_msgs::msg::Wrench::SharedPtr msg);

    // -- Service callbacks -- Implementation in ServiceCallbackImpl.cpp 

    // void getControlModesCallback(const std::shared_ptr<rmw_request_id_t> request_header,
    //                              const std::shared_ptr<yarp_control_msgs::srv::GetControlModes::Request> request,
    //                              std::shared_ptr<yarp_control_msgs::srv::GetControlModes::Response> response);
    // void getPositionCallback(const std::shared_ptr<rmw_request_id_t> request_header,
    //                          const std::shared_ptr<yarp_control_msgs::srv::GetPosition::Request> request,
    //                          std::shared_ptr<yarp_control_msgs::srv::GetPosition::Response> response);
    // void getVelocityCallback(const std::shared_ptr<rmw_request_id_t> request_header,
    //                          const std::shared_ptr<yarp_control_msgs::srv::GetVelocity::Request> request,
    //                          std::shared_ptr<yarp_control_msgs::srv::GetVelocity::Response> response);
    // void setControlModesCallback(const std::shared_ptr<rmw_request_id_t> request_header,
    //                              const std::shared_ptr<yarp_control_msgs::srv::SetControlModes::Request> request,
    //                              std::shared_ptr<yarp_control_msgs::srv::SetControlModes::Response> response);
    // void getJointsNamesCallback(const std::shared_ptr<rmw_request_id_t> request_header,
    //                             const std::shared_ptr<yarp_control_msgs::srv::GetJointsNames::Request> request,
    //                             std::shared_ptr<yarp_control_msgs::srv::GetJointsNames::Response> response);
    // void getAvailableModesCallback(const std::shared_ptr<rmw_request_id_t> request_header,
    //                                const std::shared_ptr<yarp_control_msgs::srv::GetAvailableControlModes::Request> request,
    //                                std::shared_ptr<yarp_control_msgs::srv::GetAvailableControlModes::Response> response);

};

} // namespace roboticslab

#endif // __CARTESIAN_CONTROL_SERVER_HPP__
