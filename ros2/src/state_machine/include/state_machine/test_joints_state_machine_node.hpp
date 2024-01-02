#pragma once
#include "state_machine/test_joints_state_machine.hpp"
#include "state_machine/state_machine_node.hpp"
#include "march_shared_msgs/msg/exo_state_and_joint.hpp"

class TestJointsStateMachineNode : public StateMachineNode 
{
public:
    explicit TestJointsStateMachineNode();

private:
    void handleGetExoStateArray(const std::shared_ptr<march_shared_msgs::srv::GetExoStateArray::Request> request,
        std::shared_ptr<march_shared_msgs::srv::GetExoStateArray::Response> response);
    void fillExoStateArray(march_shared_msgs::srv::GetExoStateArray_Response::SharedPtr response) const;

    rclcpp::Publisher<march_shared_msgs::msg::ExoStateAndJoint>::SharedPtr m_state_publisher;

    TestJointsStateMachine m_joint_state_machine;
};