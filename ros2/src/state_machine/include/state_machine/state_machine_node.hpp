//
// Created by marco on 13-2-23.
//

#ifndef BUILD_STATE_MACHINE_NODE_H
#define BUILD_STATE_MACHINE_NODE_H
#include "march_shared_msgs/msg/gait_type.hpp"
#include "march_shared_msgs/msg/ipd_input.hpp"
#include "march_shared_msgs/srv/gait_command.hpp"
#include "rclcpp/rclcpp.hpp"
#include "state_machine/state_machine.hpp"
#include <chrono>
#include <cstdio>
#include <march_shared_msgs/msg/error.hpp>
#include <string>

class StateMachineNode : public rclcpp::Node {
public:
    StateMachineNode();

private:
    void gait_command_callback(march_shared_msgs::msg::GaitType::SharedPtr msg);

    bool send_request(exoState desired_state);
    void response_callback(rclcpp::Client<march_shared_msgs::srv::GaitCommand>::SharedFuture response);

    rclcpp::Subscription<march_shared_msgs::msg::GaitType>::SharedPtr m_gait_command_subscriber;
    rclcpp::Client<march_shared_msgs::srv::GaitCommand>::SharedPtr m_client;
    rclcpp::Client<march_shared_msgs::srv::GaitCommand>::SharedFuture m_future;
    march_shared_msgs::srv::GaitCommand::Request::SharedPtr m_request;

    StateMachine m_state_machine;
};

#endif // BUILD_STATE_MACHINE_NODE_H
