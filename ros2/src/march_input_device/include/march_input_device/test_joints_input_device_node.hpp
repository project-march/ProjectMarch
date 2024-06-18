//
// Created by andrew on 23-11-23.
//

#pragma once
#include "rclcpp/rclcpp.hpp"
#include "march_input_device/test_joints_input_device.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "march_shared_msgs/msg/exo_mode_array.hpp"
#include "march_shared_msgs/srv/get_exo_mode_array.hpp"

class TestJointsInputDeviceNode : public rclcpp::Node {
public:
    explicit TestJointsInputDeviceNode();

private:
    rclcpp::Client<march_shared_msgs::srv::GetExoModeArray>::SharedPtr m_get_exo_mode_array_client;
    
    void sendNewModeAndJoint(const ExoMode& desired_mode);
    ExoMode askMode() const;
    std::string askJoint() const;

    TestJointsIPD m_ipd;
    std::shared_ptr<march_shared_msgs::srv::GetExoModeArray::Request> m_request;

    bool ipd_new_terminal; // Parameter to control whether a new terminal is opened or not


};