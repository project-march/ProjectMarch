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
    m_reset_publisher = this->create_publisher<std_msgs::msg::Int32>("/trajectory_reset_gate", 10);
    m_gait_response_publisher
        = this->create_publisher<march_shared_msgs::msg::GaitResponse>("/march/gait_response", 10);
    m_gait_request_subscriber = this->create_subscription<march_shared_msgs::msg::GaitRequest>(
        "/march/gait_request", 10, std::bind(&StateMachineNode::gait_command_callback, this, _1));
    m_footstep_client = this->create_client<march_shared_msgs::srv::RequestFootsteps>("footstep_generator");

    m_gait_client = this->create_client<march_shared_msgs::srv::RequestGait>("gait_selection");

    m_state_machine = StateMachine();
    m_footstep_request = std::make_shared<march_shared_msgs::srv::RequestFootsteps::Request>();
    m_gait_request = std::make_shared<march_shared_msgs::srv::RequestGait::Request>();
    while (!m_gait_client->wait_for_service(1s)) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(rclcpp::get_logger("state_machine"), "Interrupted while waiting for the service. Exiting.");
            return;
        }
        RCLCPP_INFO(rclcpp::get_logger("state_machine"), "gait_selection service not available, waiting again...");
    }
    RCLCPP_INFO(rclcpp::get_logger("state_machine"), "gait_selection service connected");

    while (!m_footstep_client->wait_for_service(1s)) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(rclcpp::get_logger("state_machine"), "Interrupted while waiting for the service. Exiting.");
            return;
        }
        RCLCPP_INFO(rclcpp::get_logger("state_machine"), "footstep_planner service not available, waiting again...");
    }
    RCLCPP_INFO(rclcpp::get_logger("state_machine"), "footstep_planner service connected");
}

/**
 * Response callback listens to see if a response from the server is received, on the request.
 * If this is the case, it is logged that the request was successful.
 *
 * If something went wrong this is also logged and the safety node should be notified.
 * @param response
 */
void StateMachineNode::response_footstep_callback(
    const rclcpp::Client<march_shared_msgs::srv::RequestFootsteps>::SharedFuture future)
{
    if (future.get()->status) {
        RCLCPP_INFO(rclcpp::get_logger("state_machine"), "Footstep request received successful!");
    } else {
        RCLCPP_ERROR(rclcpp::get_logger("state_machine"), "Footstep request was not a success!");
    }
}

/**
 * Response callback listens to see if a response from the server is received, on the request.
 * If this is the case, it is logged that the request was successful.
 *
 * If something went wrong this is also logged and the safety node should be notified.
 * @param response
 */
void StateMachineNode::response_gait_callback(
    const rclcpp::Client<march_shared_msgs::srv::RequestGait>::SharedFuture future)
{
    if (future.get()->status) {
        RCLCPP_INFO(rclcpp::get_logger("state_machine"), "Gait request received successful!");
    } else {
        RCLCPP_ERROR(rclcpp::get_logger("state_machine"), "Request was not a success!");
    }
}

/**
 * This function sends the request to the server, to set the wanted gait-type for the exo to perform.
 *
 * @param desired_state
 * @return succes of failure
 */
void StateMachineNode::send_request(exoState desired_state)
{
    int requested_gait = (int)desired_state;
    int cur_st = this->m_state_machine.get_current_state();
    auto reset_msg = std_msgs::msg::Int32();
    if (requested_gait == 1) {
        if (this->m_state_machine.get_current_state() == 0) {
            m_gait_request->home = false;
        } else {
            m_gait_request->home = true;
        }
        reset_msg.data = -1;
        m_reset_publisher->publish(reset_msg);
        m_gait_request->gait_type = 1;
        m_gait_future = m_gait_client->async_send_request(
            m_gait_request, std::bind(&StateMachineNode::response_gait_callback, this, _1));
    } else if (requested_gait == 0) {
        if (this->m_state_machine.get_current_state() == 0) {
            m_gait_request->home = true;
        } else {
            m_gait_request->home = false;
        }
        reset_msg.data = -1;
        m_reset_publisher->publish(reset_msg);
        m_gait_request->gait_type = 0;
        m_gait_future = m_gait_client->async_send_request(
            m_gait_request, std::bind(&StateMachineNode::response_gait_callback, this, _1));
    } else if (requested_gait == 5) {
        // NOTE: Make sure that everything of hte high level control and the gait selection stops.
        return;
    } else {
        //        reset_msg.data = 1;
        m_reset_publisher->publish(reset_msg);
        m_footstep_request->gait_type = requested_gait;
        m_footstep_future = m_footstep_client->async_send_request(
            m_footstep_request, std::bind(&StateMachineNode::response_footstep_callback, this, _1));
    }
}

/**
 * When the statemachine node received a command from the ipd, a transition is performed,
 * then the request for the new gait is send.
 * @param msg
 */
void StateMachineNode::gait_command_callback(march_shared_msgs::msg::GaitRequest::SharedPtr msg)
{
    auto response_msg = march_shared_msgs::msg::GaitResponse();
    if (m_state_machine.isValidTransition((exoState)msg->gait_type)) {
        response_msg.gait_type = msg->gait_type;
        m_gait_response_publisher->publish(response_msg);
        send_request((exoState)msg->gait_type);
        m_state_machine.performTransition((exoState)msg->gait_type);
    } else {
        response_msg.gait_type = m_state_machine.get_current_state();
        m_gait_response_publisher->publish(response_msg);
    }
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
