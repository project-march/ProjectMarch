/*
 * Project MARCH IX, 2023-2024
 * Author: Alexander James Becoy @alexanderjamesbecoy
 */

#include "march_ik_solver/ik_manager_node.hpp"

IKManagerNode::IKManagerNode() : Node("ik_manager_node") {
    configureStackOfTasks();

    m_exo_mode_sub = this->create_subscription<march_shared_msgs::msg::ExoMode>(
        "exo_mode", 10, std::bind(&IKManagerNode::exoModeCallback, this, std::placeholders::_1));
    m_iks_command_pub = this->create_publisher<march_shared_msgs::msg::IksCommand>("iks_command", 10);

    RCLCPP_INFO(this->get_logger(), "IK Manager Node has been initialized.");
}

void IKManagerNode::exoModeCallback(const march_shared_msgs::msg::ExoMode::SharedPtr msg) {
    publishIksCommand(msg->mode);
}

void IKManagerNode::publishIksCommand(const int iks_command) {
    try {
        march_shared_msgs::msg::IksCommand msg;
        msg.task_names = m_exo_mode_to_task_stack_map[iks_command];
        m_iks_command_pub->publish(msg);

        RCLCPP_INFO(this->get_logger(), "Updating stack of tasks in ik_solver_node with %d tasks.", msg.task_names.size());
        for (unsigned long int i = 0; i < msg.task_names.size(); i++) {
            RCLCPP_INFO(this->get_logger(), "Task %lu: %s", i, msg.task_names[i].c_str());
        }
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Failed to publish IKS command: %s", e.what());
    }
}

void IKManagerNode::configureStackOfTasks()
{

}

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<IKManagerNode>());
    rclcpp::shutdown();
    return 0;
}