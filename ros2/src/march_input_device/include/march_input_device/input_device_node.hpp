//
// Created by andrew on 23-11-23.
//

#pragma once
#include "rclcpp/rclcpp.hpp"
#include "march_input_device/input_device.hpp"
#include "std_msgs/msg/int32.hpp"
#include "march_shared_msgs/msg/exo_state_array.hpp"
#include "march_shared_msgs/srv/get_exo_state_array.hpp"

class inputDeviceNode : public rclcpp::Node {
public:
    explicit inputDeviceNode();

private:
    rclcpp::Client<march_shared_msgs::srv::GetExoStateArray>::SharedPtr m_get_exo_state_array_client;
    
    void sendNewState(const exoState& desired_state);
    exoState askState() const;

    IPD m_ipd;

    bool ipd_new_terminal; // Parameter to control whether a new terminal is opened or not


};