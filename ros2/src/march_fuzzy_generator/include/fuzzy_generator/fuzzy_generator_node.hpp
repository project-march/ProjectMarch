#ifndef MARCH_FUZZY_NODE_HPP
#define MARCH_FUZZY_NODE_HPP
#include "fuzzy_generator/fuzzy_generator.hpp"
#include "march_shared_msgs/msg/exo_mode.hpp"
#include "march_shared_msgs/msg/feet_height_stamped.hpp"
#include "march_shared_msgs/msg/joint_motor_controller_state.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include <std_msgs/msg/string.hpp>
#include <chrono>


class FuzzyGeneratorNode : public rclcpp::Node {
    public:
        FuzzyGeneratorNode();

    private:
        FuzzyGenerator m_fuzzy_generator;
        rclcpp::Subscription<march_shared_msgs::msg::FeetHeightStamped>::SharedPtr m_foot_height_subscription;
        rclcpp::Subscription<march_shared_msgs::msg::JointMotorControllerState>::SharedPtr m_torque_subscription;        
        rclcpp::Subscription<march_shared_msgs::msg::ExoMode>::SharedPtr m_mode_subscription;
        rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr m_weight_publisher;
        rclcpp::TimerBase::SharedPtr m_timer; 

        march_shared_msgs::msg::FeetHeightStamped::SharedPtr m_latest_foot_heights;
        double m_left_ankle_torque;
        double m_right_ankle_torque;
        
        static constexpr std::size_t m_joint_name_index = 0;
        static constexpr std::size_t m_position_weight_index = 1;
        static constexpr std::size_t m_torque_weight_index = 2;

        double getSpecificJointTorque(const march_shared_msgs::msg::JointMotorControllerState::SharedPtr msg, const std::string& joint_name);
        void footHeightsCallback(const march_shared_msgs::msg::FeetHeightStamped::SharedPtr msg);
        void currentModeCallback(const march_shared_msgs::msg::ExoMode::SharedPtr msg);
        void measuredTorqueCallback(const march_shared_msgs::msg::JointMotorControllerState::SharedPtr msg);
        void publishFuzzyWeights();
};
#endif // MARCH_FUZZY_NODE_HPP