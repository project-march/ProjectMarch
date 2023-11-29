#pragma once 

#include "rclcpp/rclcpp.hpp" 
#include "march_gait_planning/gait_planning.hpp"
#include "march_shared_msgs/msg/iks_foot_positions.hpp"
#include "march_shared_msgs/msg/exo_state.hpp"
#include "std_msgs/msg/int32.hpp"


class GaitPlanningNode:public rclcpp::Node {
    public: 
    explicit GaitPlanningNode();

    private: 
    rclcpp::Publisher<march_shared_msgs::msg::IksFootPositions>::SharedPtr m_iks_foot_positions_publisher; 
    rclcpp::Subscription<march_shared_msgs::msg::ExoState>::SharedPtr m_exo_state_subscriber; // exo_state == gait_type 
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr m_current_stance_foot_subscriber; 

    void currentStateCallback(const march_shared_msgs::msg::ExoState::SharedPtr msg); 
    void currentStanceFootCallback(const std_msgs::msg::Int32::SharedPtr msg); 

    GaitPlanning m_gait_planning; 

};