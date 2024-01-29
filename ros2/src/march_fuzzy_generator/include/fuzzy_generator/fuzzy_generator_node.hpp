//
// Created by rixt on 12-4-23.
//

#ifndef MARCH_FUZZY_NODE_HPP
#define MARCH_FUZZY_NODE_HPP
#include "fuzzy_generator.hpp"
#include "march_shared_msgs/msg/weight_stamped.hpp"
#include <std_msgs/msg/string.hpp>
#include <chrono>

class FuzzyGeneratorNode : public rclcpp::Node {
    public:
        FuzzyGeneratorNode();

    private:
        rclcpp::Subscription<march_shared_msgs::msg::FeetHeightStamped>::SharedPtr m_foot_height_subscription;
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr m_control_type_subscription;
        rclcpp::Publisher<march_shared_msgs::msg::WeightStamped>::SharedPtr m_weight_publisher;

        void height_callback(march_shared_msgs::msg::FeetHeightStamped::SharedPtr msg);
        void control_type_callback(std_msgs::msg::String::SharedPtr msg);
        void publish_weights(march_shared_msgs::msg::WeightStamped msg);

        FuzzyGenerator m_fuzzy_generator;
};
#endif // MARCH_FUZZY_NODE_HPP