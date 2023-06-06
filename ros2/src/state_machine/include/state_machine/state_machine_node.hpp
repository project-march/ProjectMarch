//
// Created by marco on 13-2-23.
//

#ifndef BUILD_STATE_MACHINE_NODE_H
#define BUILD_STATE_MACHINE_NODE_H
#include "march_shared_msgs/msg/gait_request.hpp"
#include "march_shared_msgs/msg/gait_response.hpp"
#include "march_shared_msgs/srv/gait_command.hpp"
#include "march_shared_msgs/srv/request_footsteps.hpp"
#include "march_shared_msgs/srv/request_gait.hpp"
#include "rclcpp/rclcpp.hpp"
#include "state_machine/state_machine.hpp"
#include "std_msgs/msg/int32.hpp"
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
    void response_footstep_callback(
        const rclcpp::Client<march_shared_msgs::srv::RequestFootsteps>::SharedFuture future);
    void response_gait_callback(const rclcpp::Client<march_shared_msgs::srv::RequestGait>::SharedFuture future);

    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr m_reset_publisher;
    rclcpp::Publisher<march_shared_msgs::msg::GaitResponse>::SharedPtr m_gait_response_publisher;
    rclcpp::Subscription<march_shared_msgs::msg::GaitRequest>::SharedPtr m_gait_request_subscriber;
    rclcpp::Client<march_shared_msgs::srv::RequestFootsteps>::SharedPtr m_footstep_client;
    rclcpp::Client<march_shared_msgs::srv::RequestGait>::SharedPtr m_gait_client;
    rclcpp::Client<march_shared_msgs::srv::RequestFootsteps>::SharedFuture m_footstep_future;
    rclcpp::Client<march_shared_msgs::srv::RequestGait>::SharedFuture m_gait_future;
    march_shared_msgs::srv::RequestFootsteps::Request::SharedPtr m_footstep_request;
    march_shared_msgs::srv::RequestGait::Request::SharedPtr m_gait_request;

    StateMachine m_state_machine;
};

#endif // BUILD_STATE_MACHINE_NODE_H
