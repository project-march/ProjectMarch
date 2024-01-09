/*
Authors: Femke Buiks, MIX

This is the header file for the GaitPlanningAnglesNode class. 

*/ 

#include "rclcpp/rclcpp.hpp" 
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"
#include "march_gait_planning/gait_planning_joint_angles.hpp"
#include "march_shared_msgs/msg/exo_state.hpp"
#include "march_shared_msgs/msg/state_estimation.hpp"
#include <vector>
#include <array> 
#include <iostream> 
#include <fstream> 
#include <sstream> 
#include <cmath>

#include "march_shared_msgs/srv/get_current_joint_positions.hpp"

class GaitPlanningAnglesNode:public rclcpp::Node 
{
public: 
    explicit GaitPlanningAnglesNode(); 

private: 
    // std::vector<double> getCurrentJointAngles();

    // Callback for the current exoState
    void currentStateCallback(const march_shared_msgs::msg::ExoState::SharedPtr msg);

    // Callback for current joint angles
    void currentJointAnglesCallback(const march_shared_msgs::msg::StateEstimation::SharedPtr msg); 

    // Functions to generalize and refactor code 
    void initializeConstantsPoints(trajectory_msgs::msg::JointTrajectoryPoint &point); 

    // Process various types of gaits 
    void processHomeStandGait();
    void processMovingGaits(const int &counter);     
    
    // Final joint angle trajectory publisher
    void publishJointTrajectoryPoints(); 

    // Member variables 
    rclcpp::Subscription<march_shared_msgs::msg::ExoState>::SharedPtr m_exo_state_subscriber; 
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr m_joint_angle_trajectory_publisher; 
    rclcpp::TimerBase::SharedPtr m_timer;
    // rclcpp::Client<march_shared_msgs::srv::GetCurrentJointPositions>::SharedPtr m_get_current_joint_angles_client;
    rclcpp::Subscription<march_shared_msgs::msg::StateEstimation>::SharedPtr m_current_state_subscriber; 
    
    
    trajectory_msgs::msg::JointTrajectory m_joints_msg;
    trajectory_msgs::msg::JointTrajectoryPoint m_trajectory_prev_point;
    trajectory_msgs::msg::JointTrajectoryPoint m_trajectory_des_point;

    std::vector<std::vector<double>> m_current_trajectory; 

    GaitPlanningAngles m_gait_planning; 
    std::vector<double> m_incremental_steps_to_home_stand;
    bool m_first_stand;

};
