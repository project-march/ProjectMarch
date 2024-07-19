#ifndef GAIN_SCHEDULER_NODE_HPP
#define GAIN_SCHEDULER_NODE_HPP
#include "gain_scheduler/gain_scheduler.hpp"
#include "march_shared_msgs/msg/joint_gains.hpp"     
#include "march_shared_msgs/msg/exo_mode.hpp"
#include "march_shared_msgs/msg/state_estimation.hpp"
#include <std_msgs/msg/string.hpp>

class GainSchedulerNode : public rclcpp::Node {
    public: 
        explicit GainSchedulerNode();

    private: 
        rclcpp::Publisher<march_shared_msgs::msg::JointGains>::SharedPtr m_joint_gains_publisher; 
        rclcpp::Subscription<march_shared_msgs::msg::ExoMode>::SharedPtr m_mode_subscriber;
        rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr m_joint_states_subscriber;
        rclcpp::Subscription<march_shared_msgs::msg::StateEstimation>::SharedPtr m_state_estimation_subscriber;
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr m_robot_description_subscriber;
        sensor_msgs::msg::JointState::SharedPtr m_latest_joint_state;
        rclcpp::TimerBase::SharedPtr m_timer; 
        uint8_t m_current_stance_leg;
        std_msgs::msg::String::SharedPtr m_robot_description;

        GainScheduler m_gain_scheduler;

        void currentModeCallback(const march_shared_msgs::msg::ExoMode::SharedPtr msg);   
        void jointStatesCallback(const sensor_msgs::msg::JointState::SharedPtr msg);
        void stateEstimationCallback(const march_shared_msgs::msg::StateEstimation::SharedPtr msg);
        void robotDescriptionCallback(const std_msgs::msg::String::SharedPtr msg);
        joints_with_gains getJointsWithGains();
        void publishJointsWithGains();
};

#endif // GAIN_SCHEDULER_NODE_HPP
