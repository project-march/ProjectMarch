//
// Created by andrew on 23-11-23.
//

#pragma once
#include "rclcpp/rclcpp.hpp"
#include "march_input_device/input_device.hpp"
#include "std_msgs/msg/int32.hpp"

class inputDeviceNode : public rclcpp::Node {
public:
    explicit inputDeviceNode();

private:
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr m_new_state_publisher;
    IPD m_ipd;
};