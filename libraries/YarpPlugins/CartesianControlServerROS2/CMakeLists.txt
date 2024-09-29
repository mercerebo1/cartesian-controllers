yarp_prepare_plugin(CartesianControlServerROS2
                    CATEGORY device
                    TYPE roboticslab::CartesianControlServerROS2
                    INCLUDE CartesianControlServerROS2.hpp
                    EXTRA_CONFIG WRAPPER=CartesianControlServerROS2
                    DEFAULT ON)

if(NOT SKIP_CartesianControlServerROS2)

    yarp_add_plugin(CartesianControlServerROS2 CartesianControlServerROS2.hpp
                                               CartesianControlServerROS2.cpp
                                               PeriodicThreadImpl.cpp
                                               DeviceDriverImpl.cpp
                                               LogComponent.hpp
                                               LogComponent.cpp)

    target_link_libraries(CartesianControlServerROS2 YARP::YARP_os
                                                     YARP::YARP_dev
                                                     rclcpp::rclcpp
                                                     std_msgs::std_msgs__rosidl_typesupport_cpp
                                                     geometry_msgs::geometry_msgs__rosidl_typesupport_cpp
                                                     #yarp_control_msgs::yarp_control_msgs__rosidl_typesupport_cpp
                                                     ROBOTICSLAB::KinematicsDynamicsInterfaces
                                                     ${orocos_kdl_LIBRARIES})

    target_include_directories(CartesianControlServerROS2 PRIVATE ${orocos_kdl_INCLUDE_DIRS})

    yarp_install(TARGETS CartesianControlServerROS2
                 LIBRARY DESTINATION ${ROBOTICSLAB-CARTESIAN-CONTROL_DYNAMIC_PLUGINS_INSTALL_DIR}
                 ARCHIVE DESTINATION ${ROBOTICSLAB-CARTESIAN-CONTROL_STATIC_PLUGINS_INSTALL_DIR}
                 YARP_INI DESTINATION ${ROBOTICSLAB-CARTESIAN-CONTROL_PLUGIN_MANIFESTS_INSTALL_DIR})

endif()
