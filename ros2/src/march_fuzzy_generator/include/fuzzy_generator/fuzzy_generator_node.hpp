//
// Created by rixt on 12-4-23.
//

#ifndef MARCH_FUZZY_NODE_HPP
#define MARCH_FUZZY_NODE_HPP
#include "fuzzy_generator/fuzzy_generator.hpp"
#include "march_shared_msgs/msg/fuzzy_weights.hpp"
#include "march_shared_msgs/msg/exo_mode.hpp"
#include <std_msgs/msg/string.hpp>
#include <chrono>

class FuzzyGeneratorNode : public rclcpp::Node {
    public:
        FuzzyGeneratorNode();

    private:
        rclcpp::Subscription<march_shared_msgs::msg::FootHeights>::SharedPtr m_foot_height_subscription;
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr m_control_type_subscription;
        rclcpp::Subscription<march_shared_msgs::msg::ExoMode>::SharedPtr m_mode_subscription;
        rclcpp::Publisher<march_shared_msgs::msg::FuzzyWeights>::SharedPtr m_weight_publisher;
        rclcpp::TimerBase::SharedPtr m_timer; 

        march_shared_msgs::msg::FootHeights::SharedPtr m_latest_foot_heights;
        std::string m_control_type = "position"; // default value

        void footHeightsCallback(const march_shared_msgs::msg::FootHeights::SharedPtr msg);
        void controlTypeCallback(std_msgs::msg::String::SharedPtr msg);
        void currentModeCallback(const march_shared_msgs::msg::ExoMode::SharedPtr msg);
        void publishFuzzyWeights();
        void timerCallback();

        FuzzyGenerator m_fuzzy_generator;
};
#endif // MARCH_FUZZY_NODE_HPP