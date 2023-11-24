//
// Created by andrew on 23-11-23.
//

#pragma once
#include "rclcpp/rclcpp.hpp"
#include "march_input_device/input_device.hpp"
#include "std_msgs/msg/int32.hpp"
#include "march_shared_msgs/msg/exo_state_array.hpp"

class inputDeviceNode : public rclcpp::Node {
public:
    explicit inputDeviceNode();
    ~inputDeviceNode();

private:
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr m_new_state_publisher;
    rclcpp::Subscription<march_shared_msgs::msg::ExoStateArray>::SharedPtr m_exo_state_array_subscriber;

    void availableStatesCallback(const march_shared_msgs::msg::ExoStateArray::SharedPtr msg);
    void sendNewState(const exoState& desired_state);

    IPD m_ipd;
    pid_t m_terminal_pid; // Process ID of the terminal that is opened
};