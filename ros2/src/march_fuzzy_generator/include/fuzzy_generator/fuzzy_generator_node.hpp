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
        FuzzyGenerator m_fuzzy_generator;
        rclcpp::Subscription<march_shared_msgs::msg::FootHeights>::SharedPtr m_foot_height_subscription;
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr m_control_type_subscription;
        rclcpp::Subscription<march_shared_msgs::msg::ExoMode>::SharedPtr m_mode_subscription;
        rclcpp::Publisher<march_shared_msgs::msg::FuzzyWeights>::SharedPtr m_weight_publisher;
        rclcpp::TimerBase::SharedPtr m_timer; 

        std::string m_control_type = "position"; // default value
        march_shared_msgs::msg::FootHeights::SharedPtr m_latest_foot_heights;
        
        static constexpr std::size_t m_joint_name_index = 0;
        static constexpr std::size_t m_position_weight_index = 1;
        static constexpr std::size_t m_torque_weight_index = 2;

        void footHeightsCallback(const march_shared_msgs::msg::FootHeights::SharedPtr msg);
        void controlTypeCallback(std_msgs::msg::String::SharedPtr msg);
        void currentModeCallback(const march_shared_msgs::msg::ExoMode::SharedPtr msg);
        void publishFuzzyWeights();

};
#endif // MARCH_FUZZY_NODE_HPP