//
// Created by Marco Bak march8 on 1-2-23.
//
#include "state_machine/state_machine_node.hpp"
#include "state_machine/state_machine.hpp"
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
    : Node("state_machine_node")
{
    m_gait_command_subscriber = this->create_subscription<march_shared_msgs::msg::GaitRequest>(
        "gait_request", 10, std::bind(&StateMachineNode::gait_command_callback, this, _1));
    m_client = this->create_client<march_shared_msgs::srv::GaitCommand>("gait_command_client");

    m_state_machine = StateMachine();

    while (!m_client->wait_for_service(1s)) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
            return;
        }
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
    }
}

/**
 * Response callback listens to see if a response from the server is received, on the request.
 * If this is the case, it is logged that the request was successful.
 *
 * If something went wrong this is also logged and the safety node should be notified.
 * @param response
 */
void StateMachineNode::response_callback(rclcpp::Client<march_shared_msgs::srv::GaitCommand>::SharedFuture response)
{
    if (response.get()->success) {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Request received successful!");
    } else {
        RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Request was not a success!");
        // do ERROR STUFF
    }
}

/**
 * This function sends the request to the server, to set the wanted gait-type for the exo to perform.
 *
 * @param desired_state
 * @return succes of failure
 */
bool StateMachineNode::send_request(exoState desired_state)
{
    m_request->gait_type = (int)desired_state;
    m_future = m_client->async_send_request(m_request, std::bind(&StateMachineNode::response_callback, this, _1));
    return m_future.get()->success;
}

/**
 * When the statemachine node received a command from the ipd, a transition is performed,
 * then the request for the new gait is send.
 * @param msg
 */
void StateMachineNode::gait_command_callback(march_shared_msgs::msg::GaitRequest::SharedPtr msg)
{
    m_state_machine.performTransition((exoState)msg->gait_type);
    send_request(m_state_machine.get_current_state());
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
