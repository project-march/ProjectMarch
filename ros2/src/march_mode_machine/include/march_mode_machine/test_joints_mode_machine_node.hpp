#pragma once
#include "march_mode_machine/test_joints_mode_machine.hpp"
#include "march_shared_msgs/msg/exo_mode_and_joint.hpp"
#include "march_shared_msgs/msg/gait_request.hpp"
#include "march_shared_msgs/msg/gait_response.hpp"
#include "march_shared_msgs/srv/gait_command.hpp"
#include "march_shared_msgs/srv/request_footsteps.hpp"
#include "march_shared_msgs/srv/request_gait.hpp"
#include "rclcpp/rclcpp.hpp"
#include "march_mode_machine/mode_machine.hpp"
#include "std_msgs/msg/int32.hpp"
#include <chrono>
#include <cstdio>
#include <march_shared_msgs/msg/error.hpp>
#include <string>
#include "march_shared_msgs/msg/exo_mode_array.hpp"
#include "march_shared_msgs/srv/get_exo_mode_array.hpp"

class TestJointsModeMachineNode :  public rclcpp::Node
{
public:
    explicit TestJointsModeMachineNode();
    ~TestJointsModeMachineNode();

private:
    void sendRequest(const ExoMode& desired_mode);
    void responseFootstepCallback(
        const rclcpp::Client<march_shared_msgs::srv::RequestFootsteps>::SharedFuture future);
    void responseGaitCallback(const rclcpp::Client<march_shared_msgs::srv::RequestGait>::SharedFuture future);
    void newModeCallback(const std_msgs::msg::Int32::SharedPtr msg);
    
    void publishAvailableExoModes(march_shared_msgs::msg::ExoModeArray::SharedPtr msg) const;

    void handleGetExoModeArray(const std::shared_ptr<march_shared_msgs::srv::GetExoModeArray::Request> request,
        std::shared_ptr<march_shared_msgs::srv::GetExoModeArray::Response> response);
    void fillExoModeArray(march_shared_msgs::srv::GetExoModeArray_Response::SharedPtr response) const;

    rclcpp::Publisher<march_shared_msgs::msg::ExoModeAndJoint>::SharedPtr m_mode_publisher;
    rclcpp::Service<march_shared_msgs::srv::GetExoModeArray>::SharedPtr m_get_exo_mode_array_service;

    TestJointsModeMachine m_mode_machine;
};