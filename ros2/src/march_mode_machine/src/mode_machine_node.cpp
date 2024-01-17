//
// Created by Marco Bak march8 on 1-2-23.
//
#include "march_mode_machine/mode_machine_node.hpp"
#include "march_mode_machine/mode_machine.hpp"
#include <functional>
using std::placeholders::_1;
using std::placeholders::_2;
using namespace std::chrono_literals;

/**
 * Creates the node for the mode-machine to communicate with the rest of the codebase.
 *
 * This exist of:
 * the client, that communicates with the gait loader server;
 * gait_command_subscriber, communicates with the ipd to get the selected gatType.
 * The mode machine node also has an modemachine to validate the wanted transitions.
 */
ModeMachineNode::ModeMachineNode()
    : Node("mode_machine_node"),
    m_mode_machine(ModeMachine())
{
    m_get_exo_mode_array_service = create_service<march_shared_msgs::srv::GetExoModeArray>(
        "get_exo_mode_array",
        std::bind(&ModeMachineNode::handleGetExoModeArray, this, _1, _2));

    m_mode_publisher = create_publisher<march_shared_msgs::msg::ExoMode>("current_mode", 10);

    RCLCPP_WARN(rclcpp::get_logger("mode_machine"), "Mode Machine Node succesfully initialized");
}

ModeMachineNode::~ModeMachineNode()
{
    RCLCPP_WARN(rclcpp::get_logger("mode_machine"), "Deconstructor of Mode Machine node called");
}


void ModeMachineNode::fillExoModeArray(march_shared_msgs::srv::GetExoModeArray_Response::SharedPtr response) const
{
    std::set<exoMode> available_modes = m_mode_machine.getAvailableModes((exoMode)m_mode_machine.getCurrentMode());

    // Clear the existing modes in the msg
    response->mode_array.modes.clear();

    // Iterate over the available_modes set and add each mode to the msg
    for (const auto& mode : available_modes) {
        march_shared_msgs::msg::ExoMode exoModeMsg;
        exoModeMsg.mode = static_cast<int8_t>(mode);
        response->mode_array.modes.push_back(exoModeMsg);
    }
}

void ModeMachineNode::handleGetExoModeArray(const std::shared_ptr<march_shared_msgs::srv::GetExoModeArray::Request> request,
        std::shared_ptr<march_shared_msgs::srv::GetExoModeArray::Response> response)
{
    RCLCPP_INFO(rclcpp::get_logger("mode_machine"), "Request received!");
    exoMode new_mode = (exoMode)request->desired_mode.mode;
    if (m_mode_machine.isValidTransition(new_mode))
    {
        m_mode_machine.performTransition(new_mode);
        auto mode_msg = march_shared_msgs::msg::ExoMode();
        mode_msg.mode = m_mode_machine.getCurrentMode();
        m_mode_publisher->publish(mode_msg);
    } else 
    {
        RCLCPP_WARN(rclcpp::get_logger("mode_machine"), "Invalid mode transition! Ignoring new mode.");
    }
    fillExoModeArray(response);
    RCLCPP_INFO(rclcpp::get_logger("mode_machine"), "Response sent!");
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
    rclcpp::spin(std::make_shared<ModeMachineNode>());

    rclcpp::shutdown();
    return 0;
}
