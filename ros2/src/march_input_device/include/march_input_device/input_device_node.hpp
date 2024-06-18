//
// Created by andrew on 23-11-23.
//

#pragma once
#include "rclcpp/rclcpp.hpp"
#include "march_input_device/input_device.hpp"
#include "std_msgs/msg/int32.hpp"
#include "march_shared_msgs/msg/exo_mode_array.hpp"
#include "march_shared_msgs/srv/get_exo_mode_array.hpp"

class inputDeviceNode : public rclcpp::Node {
public:
    explicit inputDeviceNode();

private:
    rclcpp::Client<march_shared_msgs::srv::GetExoModeArray>::SharedPtr m_get_exo_mode_array_client;
    
    void sendNewMode(const ExoMode& desired_mode);
    ExoMode askMode() const;

    IPD m_ipd;

    bool ipd_new_terminal; // Parameter to control whether a new terminal is opened or not


};