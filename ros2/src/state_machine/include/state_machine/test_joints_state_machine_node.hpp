#pragma once
#include "state_machine/test_joints_state_machine.hpp"
#include "march_shared_msgs/msg/exo_state_and_joint.hpp"
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
#include "march_shared_msgs/srv/get_exo_state_array.hpp"

class TestJointsStateMachineNode :  public rclcpp::Node
{
public:
    explicit TestJointsStateMachineNode();
    ~TestJointsStateMachineNode();

private:
    void sendRequest(const exoState& desired_state);
    void responseFootstepCallback(
        const rclcpp::Client<march_shared_msgs::srv::RequestFootsteps>::SharedFuture future);
    void responseGaitCallback(const rclcpp::Client<march_shared_msgs::srv::RequestGait>::SharedFuture future);
    void newStateCallback(const std_msgs::msg::Int32::SharedPtr msg);
    
    void publishAvailableExoStates(march_shared_msgs::msg::ExoStateArray::SharedPtr msg) const;

    void handleGetExoStateArray(const std::shared_ptr<march_shared_msgs::srv::GetExoStateArray::Request> request,
        std::shared_ptr<march_shared_msgs::srv::GetExoStateArray::Response> response);
    void fillExoStateArray(march_shared_msgs::srv::GetExoStateArray_Response::SharedPtr response) const;

    rclcpp::Publisher<march_shared_msgs::msg::ExoStateAndJoint>::SharedPtr m_state_publisher;
    rclcpp::Service<march_shared_msgs::srv::GetExoStateArray>::SharedPtr m_get_exo_state_array_service;

    TestJointsStateMachine m_state_machine;
};