//
// Created by marco on 13-2-23.
//

#ifndef BUILD_STATE_MACHINE_NODE_H
#define BUILD_STATE_MACHINE_NODE_H
#include "march_shared_msgs/msg/gait_request.hpp"
#include "march_shared_msgs/msg/gait_response.hpp"
#include "march_shared_msgs/srv/gait_command.hpp"
#include "march_shared_msgs/srv/request_footsteps.hpp"
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
    void gait_command_callback(march_shared_msgs::msg::GaitRequest::SharedPtr msg);

    void send_request(exoState desired_state);
    void response_callback(const rclcpp::Client<march_shared_msgs::srv::RequestFootsteps>::SharedFuture future);

    rclcpp::Publisher<march_shared_msgs::msg::GaitResponse>::SharedPtr m_gait_response_publisher;
    rclcpp::Subscription<march_shared_msgs::msg::GaitRequest>::SharedPtr m_gait_request_subscriber;
    rclcpp::Client<march_shared_msgs::srv::RequestFootsteps>::SharedPtr m_client;
    rclcpp::Client<march_shared_msgs::srv::RequestFootsteps>::SharedFuture m_future;
    march_shared_msgs::srv::RequestFootsteps::Request::SharedPtr m_request;

    StateMachine m_state_machine;
};

#endif // BUILD_STATE_MACHINE_NODE_H
