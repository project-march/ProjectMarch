/*Authors: Femke Buiks and Andrew Hutani, MIX*/

#pragma once 

#include "rclcpp/rclcpp.hpp" 
#include "march_gait_planning/gait_planning.hpp"
#include "march_shared_msgs/msg/iks_foot_positions.hpp"
#include "march_shared_msgs/msg/state_estimation.hpp"
#include "march_shared_msgs/msg/exo_mode.hpp"
#include "std_msgs/msg/int32.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "march_shared_msgs/srv/get_current_stance_leg.hpp"


class GaitPlanningNode:public rclcpp::Node {
    public: 
    explicit GaitPlanningNode();

    private: 
    rclcpp::Publisher<march_shared_msgs::msg::IksFootPositions>::SharedPtr m_iks_foot_positions_publisher; 
    rclcpp::Subscription<march_shared_msgs::msg::ExoMode>::SharedPtr m_exo_mode_subscriber; // exo_mode == gait_type 
    rclcpp::Subscription<march_shared_msgs::msg::StateEstimation>::SharedPtr m_exo_joint_state_subscriber; 

    void currentModeCallback(const march_shared_msgs::msg::ExoMode::SharedPtr msg); 
    void currentExoJointStateCallback(const march_shared_msgs::msg::StateEstimation::SharedPtr msg); 
    void setFootPositionsMessage(double left_x, double left_y, double left_z, 
                            double right_x, double right_y, double right_z);
    void footPositionsPublish(); 

    void timerCallback();

    GaitPlanning m_gait_planning; 

    std::vector<std::array<double, 4>> m_current_trajectory; 
    march_shared_msgs::msg::IksFootPositions::SharedPtr m_desired_footpositions_msg; 
    rclcpp::TimerBase::SharedPtr m_timer;

};