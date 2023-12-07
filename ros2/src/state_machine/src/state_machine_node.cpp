//
// Created by Marco Bak march8 on 1-2-23.
//
#include "state_machine/state_machine_node.hpp"
#include "state_machine/state_machine.hpp"
#include <functional>
using std::placeholders::_1;
using std::placeholders::_2;
using namespace std::chrono_literals;

/**
 * Creates the node for the state-machine to communicate with the rest of the codebase.
 *
 * This exist of:
 * the client, that communicates with the gait loader server;
 * gait_command_subscriber, communicates with the ipd to get the selected gatType.
 * The state machine node also has an statemachine to validate the wanted transitions.
 */
StateMachineNode::StateMachineNode()
    : Node("state_machine_node"),
    m_state_machine(StateMachine())
{
    m_get_exo_state_array_service = create_service<march_shared_msgs::srv::GetExoStateArray>(
        "get_exo_state_array",
        std::bind(&StateMachineNode::handleGetExoStateArray, this, _1, _2));

    m_state_publisher = create_publisher<march_shared_msgs::msg::ExoState>("current_state", 10);

    RCLCPP_WARN(rclcpp::get_logger("state_machine"), "State Machine Node succesfully initialized");
}

void StateMachineNode::fillExoStateArray(march_shared_msgs::srv::GetExoStateArray_Response::SharedPtr response) const
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
}

void StateMachineNode::handleGetExoStateArray(const std::shared_ptr<march_shared_msgs::srv::GetExoStateArray::Request> request,
        std::shared_ptr<march_shared_msgs::srv::GetExoStateArray::Response> response)
{
    RCLCPP_INFO(rclcpp::get_logger("state_machine"), "Request received!");
    exoState new_state = (exoState)request->desired_state.state;
    if (m_state_machine.isValidTransition(new_state))
    {
        m_state_machine.performTransition(new_state);
        auto state_msg = march_shared_msgs::msg::ExoState();
        state_msg.state = m_state_machine.getCurrentState();
        m_state_publisher->publish(state_msg);
    } else 
    {
        RCLCPP_WARN(rclcpp::get_logger("state_machine"), "Invalid state transition! Ignoring new state.");
    }
    fillExoStateArray(response);
    RCLCPP_INFO(rclcpp::get_logger("state_machine"), "Response sent!");
}
/**
 * Main function to run the node.
 * @param argc
 * @param argv
 * @return
 */
int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<StateMachineNode>());

    rclcpp::shutdown();
    return 0;
}
