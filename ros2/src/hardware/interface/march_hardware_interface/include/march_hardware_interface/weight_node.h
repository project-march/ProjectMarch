//
// Created by rixt on 14-4-23.
//

#ifndef MARCH_WEIGHT_NODE_H
#define MARCH_WEIGHT_NODE_H
#include "rclcpp/rclcpp.hpp"
#include "march_shared_msgs/msg/weight_stamped.hpp"
#include "march_exo_system_interface.hpp"
#include <std_msgs/msg/string.hpp>

class WeightNode : public rclcpp::Node {
public:
    WeightNode();

private:
    rclcpp::Subscription<march_shared_msgs::msg::WeightStamped>::SharedPtr m_fuzzy_weight_subscription;

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr m_control_type_subscription;

    void fuzzy_weight_callback(march_shared_msgs::msg::WeightStamped::SharedPtr msg);

    void control_type_callback(std_msgs::msg::String::SharedPtr msg);

    void setJointsWeight(std::string leg, float position_weight, float torque_weight);

    march_hardware_interface::MarchExoSystemInterface* m_hardware_interface;
};
#endif //MARCH_WEIGHT_NODE_H
