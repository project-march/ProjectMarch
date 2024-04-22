/*
 * Project MARCH IX, 2023-2024
 * Author: Alexander James Becoy @alexanderjamesbecoy
 */

#ifndef MARCH_IK_SOLVER__IK_MANAGER_NODE_HPP_
#define MARCH_IK_SOLVER__IK_MANAGER_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <march_shared_msgs/msg/exo_mode.hpp>
#include <march_shared_msgs/msg/iks_command.hpp>

#include <string>
#include <vector>
#include <unordered_map>

typedef std::vector<std::string> StackOfTasks;

class IKManagerNode : public rclcpp::Node {
public:
    IKManagerNode();
    ~IKManagerNode() = default;

private:
    void exoModeCallback(const march_shared_msgs::msg::ExoMode::SharedPtr msg);
    void publishIksCommand(const int iks_command);

    void configureStackOfTasks();

    rclcpp::Subscription<march_shared_msgs::msg::ExoMode>::SharedPtr m_exo_mode_sub;
    rclcpp::Publisher<march_shared_msgs::msg::IksCommand>::SharedPtr m_iks_command_pub;

    std::unordered_map<int, StackOfTasks> m_exo_mode_to_task_stack_map;
};

#endif  // MARCH_IK_SOLVER__IK_MANAGER_NODE_HPP_