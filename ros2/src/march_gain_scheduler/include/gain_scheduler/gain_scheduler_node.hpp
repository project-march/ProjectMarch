#include "gain_scheduler/gain_scheduler.hpp"
#include "march_shared_msgs/msg/pid_values.hpp"     
#include "march_shared_msgs/msg/exo_mode.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

class GainSchedulerNode:public rclcpp::Node {
    
    public: 
        explicit GainSchedulerNode();

    private: 
        rclcpp::Publisher<march_shared_msgs::msg::PidValues>::SharedPtr m_pid_values_publisher; 
        rclcpp::Subscription<march_shared_msgs::msg::ExoMode>::SharedPtr m_mode_subscriber;
        rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr m_joint_states_subscriber;
        rclcpp::TimerBase::SharedPtr m_timer; 
        sensor_msgs::msg::JointState::SharedPtr m_latest_joint_state;
        GainScheduler m_gain_scheduler;

        void currentModeCallback(const march_shared_msgs::msg::ExoMode::SharedPtr msg);   
        void jointStatesCallback(const sensor_msgs::msg::JointState::SharedPtr msg);
        std::string vectorToString(const std::vector<double>& vec);
        void publishPidValues();
        void timerCallback();
};

