//
// Created by rixt on 14-4-23.
//

#ifndef MARCH_WEIGHT_NODE_H
#define MARCH_WEIGHT_NODE_H
#include "rclcpp/rclcpp.hpp"
#include "march_shared_msgs/msg/weight_stamped.hpp"
#include "march_exo_system_interface.hpp"

class WeightNode : public rclcpp::Node {
public:
    WeightNode();

private:
    rclcpp::Subscription<march_shared_msgs::msg::WeightStamped>::SharedPtr m_weight_subscription;

    void weight_callback(march_shared_msgs::msg::WeightStamped::SharedPtr msg);
};
#endif //MARCH_WEIGHT_NODE_H
