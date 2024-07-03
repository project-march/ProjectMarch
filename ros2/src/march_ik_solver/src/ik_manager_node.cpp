/*
 * Project MARCH IX, 2023-2024
 * Author: Alexander James Becoy @alexanderjamesbecoy
 */

#include "march_ik_solver/ik_manager_node.hpp"

#include <functional>

IKManagerNode::IKManagerNode() : Node("ik_manager")
{
    RCLCPP_INFO(this->get_logger(), "Initializing IK Manager Node...");
    configureStackOfTasks();

    m_exo_mode_sub = this->create_subscription<march_shared_msgs::msg::ExoMode>(
        "current_mode", 10, std::bind(&IKManagerNode::exoModeCallback, this, std::placeholders::_1));
    m_iks_command_pub = this->create_publisher<march_shared_msgs::msg::IksCommand>("ik_solver/command", 10);

    RCLCPP_INFO(this->get_logger(), "IK Manager Node has been initialized.");
}

void IKManagerNode::exoModeCallback(const march_shared_msgs::msg::ExoMode::SharedPtr msg)
{
    publishIksCommand(msg->mode);
}

void IKManagerNode::publishIksCommand(const int iks_command)
{
    try {
        march_shared_msgs::msg::IksCommand msg;
        msg.header.stamp = this->now();
        msg.exo_mode = m_exo_modes[iks_command];
        msg.task_names = m_exo_mode_to_task_stack_map[iks_command];
        m_iks_command_pub->publish(msg);

        RCLCPP_DEBUG(this->get_logger(), "Updating stack of tasks in ik_solver_node with %d tasks.", msg.task_names.size());
        for (unsigned long int i = 0; i < msg.task_names.size(); i++) {
            RCLCPP_DEBUG(this->get_logger(), "Task %lu: %s", i, msg.task_names[i].c_str());
        }
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Failed to publish IKS command: %s", e.what());
    }
}

void IKManagerNode::configureStackOfTasks()
{
    declare_parameter("exo_modes", std::vector<std::string>());
    m_exo_modes = get_parameter("exo_modes").as_string_array();
    RCLCPP_DEBUG(this->get_logger(), "Configuring stack of tasks for %lu exo modes.", m_exo_modes.size());

    for (unsigned long int i = 0; i < m_exo_modes.size(); i++) {
        std::string exo_mode_param = "stack_of_tasks." + m_exo_modes[i];
        declare_parameter(exo_mode_param, std::vector<std::string>());
        std::vector<std::string> stack_of_tasks = get_parameter(exo_mode_param).as_string_array();
        m_exo_mode_to_task_stack_map[i] = stack_of_tasks;
    }

    RCLCPP_DEBUG(this->get_logger(), "Stack of tasks has been configured. There are %lu exo modes with stack of tasks.", m_exo_mode_to_task_stack_map.size());
}

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<IKManagerNode>());
    rclcpp::shutdown();
    return 0;
}