//
// Created by Marco Bak on 23-2-23.
//

#ifndef BUILD_GAIT_COMMAND_NODE_HPP
#define BUILD_GAIT_COMMAND_NODE_HPP
#include "march_shared_msgs/msg/gait_type.hpp"
#include "march_shared_msgs/msg/ipd_input.hpp"
#include "rclcpp/rclcpp.hpp"
#include <cstdio>

#include <chrono>
#include <cstdlib>
#include <memory>

class GaitCommandNode : public rclcpp::Node {
public:
    GaitCommandNode();

private:
    void ipd_callback(march_shared_msgs::msg::IpdInput::SharedPtr msg);

    rclcpp::Subscription<march_shared_msgs::msg::IpdInput>::SharedPtr m_ipd_subscriber;
    rclcpp::Publisher<march_shared_msgs::msg::GaitType>::SharedPtr m_gait_type_publisher;
};

#endif //BUILD_GAIT_COMMAND_NODE_HPP
