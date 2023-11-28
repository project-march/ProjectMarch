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
    : Node("state_machine_node")
{
    //TODO: change all topic names to not include "/march/", to avoid grouping all march topics.
    m_reset_publisher = create_publisher<std_msgs::msg::Int32>("/trajectory_reset_gate", 10);
    m_gait_response_publisher
        = create_publisher<march_shared_msgs::msg::GaitResponse>("/march/gait_response", 10);
    m_gait_request_subscriber = create_subscription<march_shared_msgs::msg::GaitRequest>(
        "/march/gait_request", 10, std::bind(&StateMachineNode::gaitCommandCallback, this, _1));
    m_new_state_subscriber = create_subscription<std_msgs::msg::Int32>(
        "new_state", 10, std::bind(&StateMachineNode::newStateCallback, this, _1));
    m_footstep_client = create_client<march_shared_msgs::srv::RequestFootsteps>("footstep_generator");

    m_exo_state_array_publisher = create_publisher<march_shared_msgs::msg::ExoStateArray>("available_states", 10);

    m_gait_client = create_client<march_shared_msgs::srv::RequestGait>("gait_selection");

    m_state_machine = StateMachine();
    m_footstep_request = std::make_shared<march_shared_msgs::srv::RequestFootsteps::Request>();
    m_gait_request = std::make_shared<march_shared_msgs::srv::RequestGait::Request>();
    // while (!m_gait_client->wait_for_service(1s)) {
    //     if (!rclcpp::ok()) {
    //         RCLCPP_ERROR(rclcpp::get_logger("state_machine"), "Interrupted while waiting for the service. Exiting.");
    //         return;
    //     }
    //     RCLCPP_INFO(rclcpp::get_logger("state_machine"), "gait_selection service not available, waiting again...");
    // }
    // RCLCPP_INFO(rclcpp::get_logger("state_machine"), "gait_selection service connected");

    // while (!m_footstep_client->wait_for_service(1s)) {
    //     if (!rclcpp::ok()) {
    //         RCLCPP_ERROR(rclcpp::get_logger("state_machine"), "Interrupted while waiting for the service. Exiting.");
    //         return;
    //     }
    //     RCLCPP_INFO(rclcpp::get_logger("state_machine"), "footstep_planner service not available, waiting again...");
    // }
    // RCLCPP_INFO(rclcpp::get_logger("state_machine"), "footstep_planner service connected");
}

/**
 * Response callback listens to see if a response from the server is received, on the request.
 * If this is the case, it is logged that the request was successful.
 *
 * If something went wrong this is also logged and the safety node should be notified.
 * @param response
 */
void StateMachineNode::responseFootstepCallback(
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
void StateMachineNode::responseGaitCallback(
    const rclcpp::Client<march_shared_msgs::srv::RequestGait>::SharedFuture future)
{
    RCLCPP_INFO(get_logger(), "response_gait_callback");
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
void StateMachineNode::sendRequest(const exoState& desired_state)
{
    int requested_gait = (int)desired_state;
    int current_state = m_state_machine.getCurrentState();
    auto reset_msg = std_msgs::msg::Int32();
    if (requested_gait == 1) {
        if (current_state == 0) {
            m_gait_request->home = false;
        } else {
            m_gait_request->home = true;
        }
        reset_msg.data = -1;
        m_reset_publisher->publish(reset_msg);
        m_gait_request->gait_type = 1;
        m_gait_future = m_gait_client->async_send_request(
            m_gait_request, std::bind(&StateMachineNode::responseGaitCallback, this, _1));

        m_reset_publisher->publish(reset_msg);
        m_footstep_request->gait_type = requested_gait;
        m_footstep_future = m_footstep_client->async_send_request(
            m_footstep_request, std::bind(&StateMachineNode::responseFootstepCallback, this, _1));

    } else if (requested_gait == 0) {
        if (current_state == 0 || current_state == 4) {
            m_gait_request->home = true;
        } else {
            m_gait_request->home = false;
        }
        reset_msg.data = -1;
        m_reset_publisher->publish(reset_msg);
        m_gait_request->gait_type = 0;
        m_gait_future = m_gait_client->async_send_request(
            m_gait_request, std::bind(&StateMachineNode::responseGaitCallback, this, _1));
    } else if (requested_gait == 5) {
        // NOTE: Make sure that everything of hte high level control and the gait selection stops.
        return;
    } else {
        //        reset_msg.data = 1;
        m_reset_publisher->publish(reset_msg);
        m_footstep_request->gait_type = requested_gait;
        m_footstep_future = m_footstep_client->async_send_request(
            m_footstep_request, std::bind(&StateMachineNode::responseFootstepCallback, this, _1));
    }
}

/**
 * When the statemachine node received a command from the ipd, a transition is performed,
 * then the request for the new gait is send.
 * @param msg
 */
void StateMachineNode::gaitCommandCallback(march_shared_msgs::msg::GaitRequest::SharedPtr msg)
{
    auto response_msg = march_shared_msgs::msg::GaitResponse();
    if (m_state_machine.isValidTransition((exoState)msg->gait_type)) {
        response_msg.gait_type = msg->gait_type;
        m_gait_response_publisher->publish(response_msg);
        sendRequest((exoState)msg->gait_type);
        m_state_machine.performTransition((exoState)msg->gait_type);
    } else {
        response_msg.gait_type = m_state_machine.getCurrentState();
        m_gait_response_publisher->publish(response_msg);
    }
}

void StateMachineNode::fillExoStateArray(march_shared_msgs::msg::ExoStateArray::SharedPtr msg) const
{
    std::set<exoState> available_states = m_state_machine.getAvailableStates((exoState)m_state_machine.getCurrentState());

    // Clear the existing states in the msg
    msg->states.clear();

    // Iterate over the available_states set and add each state to the msg
    for (const auto& state : available_states) {
        march_shared_msgs::msg::ExoState exoStateMsg;
        exoStateMsg.state = static_cast<int8_t>(state);
        msg->states.push_back(exoStateMsg);
    }
}

void StateMachineNode::publishAvailableExoStates(march_shared_msgs::msg::ExoStateArray::SharedPtr msg) const
{
    StateMachineNode::fillExoStateArray(msg);
    m_exo_state_array_publisher->publish(*msg);
    // Iterate over the states vector and print each state
    for (const auto& state : msg->states) {
        RCLCPP_INFO(get_logger(), "Sent available state: %d", state.state);
    }
}

void StateMachineNode::newStateCallback(const std_msgs::msg::Int32::SharedPtr msg)
    {
        RCLCPP_INFO(get_logger(), "Received new state: %d", msg->data);
        if (m_state_machine.isValidTransition((exoState)msg->data)) 
        {
            sendRequest((exoState)msg->data);
            m_state_machine.performTransition((exoState)msg->data);
        } else 
        {
            RCLCPP_WARN(get_logger(), "Invalid state transition! Ignoring new state.");
        }
        march_shared_msgs::msg::ExoStateArray::SharedPtr publishMsg = std::make_shared<march_shared_msgs::msg::ExoStateArray>();  
        publishAvailableExoStates(publishMsg);
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
