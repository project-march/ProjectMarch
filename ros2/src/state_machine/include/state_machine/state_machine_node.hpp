//
// Created by marco on 13-2-23.
//
#pragma once
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
#include "march_shared_msgs/msg/exo_state_array.hpp"

class StateMachineNode : public rclcpp::Node 
{
public:
    explicit StateMachineNode();

private:
    void gaitCommandCallback(const march_shared_msgs::msg::GaitRequest::SharedPtr msg);

    void sendRequest(const exoState& desired_state);
    void responseFootstepCallback(
        const rclcpp::Client<march_shared_msgs::srv::RequestFootsteps>::SharedFuture future);
    void responseGaitCallback(const rclcpp::Client<march_shared_msgs::srv::RequestGait>::SharedFuture future);
    void newStateCallback(const std_msgs::msg::Int32::SharedPtr msg);

    void fillExoStateArray(march_shared_msgs::msg::ExoStateArray::SharedPtr msg) const;
    void publishAvailableExoStates(march_shared_msgs::msg::ExoStateArray::SharedPtr msg) const;

    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr m_reset_publisher;
    rclcpp::Publisher<march_shared_msgs::msg::GaitResponse>::SharedPtr m_gait_response_publisher;
    rclcpp::Subscription<march_shared_msgs::msg::GaitRequest>::SharedPtr m_gait_request_subscriber;
    rclcpp::Client<march_shared_msgs::srv::RequestFootsteps>::SharedPtr m_footstep_client;
    rclcpp::Client<march_shared_msgs::srv::RequestGait>::SharedPtr m_gait_client;
    rclcpp::Client<march_shared_msgs::srv::RequestFootsteps>::SharedFuture m_footstep_future;
    rclcpp::Client<march_shared_msgs::srv::RequestGait>::SharedFuture m_gait_future;
    march_shared_msgs::srv::RequestFootsteps::Request::SharedPtr m_footstep_request;
    march_shared_msgs::srv::RequestGait::Request::SharedPtr m_gait_request;

    
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr m_new_state_subscriber;
    rclcpp::Publisher<march_shared_msgs::msg::ExoStateArray>::SharedPtr m_exo_state_array_publisher;
    rclcpp::Publisher<march_shared_msgs::msg::ExoState>::SharedPtr m_state_publisher;

    StateMachine m_state_machine;
};
