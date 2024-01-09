/*Authors: Femke Buiks and Andrew Hutani, MIX*/

#pragma once 

#include "rclcpp/rclcpp.hpp" 
#include "march_gait_planning/gait_planning.hpp"
#include "march_shared_msgs/msg/iks_foot_positions.hpp"
#include "march_shared_msgs/msg/state_estimation.hpp"
#include "march_shared_msgs/msg/exo_state.hpp"
#include "std_msgs/msg/int32.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "march_shared_msgs/srv/get_current_stance_leg.hpp"


class GaitPlanningNode:public rclcpp::Node {
    public: 
    explicit GaitPlanningNode();

    private: 
    rclcpp::Publisher<march_shared_msgs::msg::IksFootPositions>::SharedPtr m_iks_foot_positions_publisher; 
    rclcpp::Subscription<march_shared_msgs::msg::ExoState>::SharedPtr m_exo_state_subscriber; // exo_state == gait_type 
    rclcpp::Subscription<march_shared_msgs::msg::StateEstimation>::SharedPtr m_exo_joint_state_subscriber; 
    
    // rclcpp::Client<march_shared_msgs::srv::GetCurrentStanceLeg>::SharedPtr m_stance_leg_client; 
    // march_shared_msgs::srv::GetCurrentStanceLeg::Request::SharedPtr m_stance_leg_request;

    void currentStateCallback(const march_shared_msgs::msg::ExoState::SharedPtr msg); 
    void currentExoJointStateCallback(const march_shared_msgs::msg::StateEstimation::SharedPtr msg); 
    void setFootPositionsMessage(double left_x, double left_y, double left_z, 
                            double right_x, double right_y, double right_z);
    void footPositionsPublish(); 

    // void responseStanceLegCallback(std::shared_future<march_shared_msgs::srv::GetCurrentStanceLeg::Response::SharedPtr> future);
    // void sendRequest(const bool& gait_complete); 

    void timerCallback();

    GaitPlanning m_gait_planning; 

    std::vector<std::array<double, 4>> m_current_trajectory; 
    march_shared_msgs::msg::IksFootPositions::SharedPtr m_desired_footpositions_msg; 
    rclcpp::TimerBase::SharedPtr m_timer;
    //Remove, not necessary anymore 
    // bool m_response_received;


};