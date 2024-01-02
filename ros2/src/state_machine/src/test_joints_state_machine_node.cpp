#include "state_machine/test_joints_state_machine_node.hpp"
#include "state_machine/test_joints_state_machine_node.hpp"

TestJointsStateMachineNode::TestJointsStateMachineNode()

{
    m_state_publisher = create_publisher<march_shared_msgs::msg::ExoStateAndJoint>("current_state", 10);
    m_state_machine = TestJointsStateMachine();
    RCLCPP_WARN(rclcpp::get_logger("state_machine"), "Joint State Machine Node succesfully initialized");
}

void TestJointsStateMachineNode::handleGetExoStateArray(const std::shared_ptr<march_shared_msgs::srv::GetExoStateArray::Request> request,
        std::shared_ptr<march_shared_msgs::srv::GetExoStateArray::Response> response)
{
    RCLCPP_INFO(rclcpp::get_logger("state_machine"), "Request received!");
    exoState new_state = (exoState)request->desired_state.state;
    if (m_state_machine.isValidTransition(new_state))
    {
        m_state_machine.performTransition(new_state);
        auto state_msg = march_shared_msgs::msg::ExoStateAndJoint();
        state_msg.state = m_state_machine.getCurrentState();
        state_msg.joint.data = request->actuated_joint.data;

        m_state_publisher->publish(state_msg);
    } else 
    {
        RCLCPP_WARN(rclcpp::get_logger("state_machine"), "Invalid state transition! Ignoring new state.");
    }
    fillExoStateArray(response);
    RCLCPP_INFO(rclcpp::get_logger("state_machine"), "Response sent!");
}

void TestJointsStateMachineNode::fillExoStateArray(march_shared_msgs::srv::GetExoStateArray_Response::SharedPtr response) const
{
    std::set<exoState> available_states = m_state_machine.getAvailableStates((exoState)m_state_machine.getCurrentState());

    // Clear the existing states in the msg
    response->state_array.states.clear();

    // Iterate over the available_states set and add each state to the msg
    for (const auto& state : available_states) {
        march_shared_msgs::msg::ExoState exoStateMsg;
        exoStateMsg.state = static_cast<int8_t>(state);
        response->state_array.states.push_back(exoStateMsg);
    }

    response->current_state.state = m_state_machine.getCurrentState();
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TestJointsStateMachineNode>());

    rclcpp::shutdown();
    return 0;
}