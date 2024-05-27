/*
Authors: Femke Buiks, MIX

This is the header file for the GaitPlanningAnglesNode class. 

*/ 

#include "rclcpp/rclcpp.hpp" 
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"
#include "march_gait_planning/gait_planning_joint_angles.hpp"
#include "march_shared_msgs/msg/exo_mode.hpp"
#include "march_shared_msgs/msg/state_estimation.hpp"
#include "../../march_mode_machine/include/march_mode_machine/exo_mode.hpp"
#include <vector>
#include <array> 
#include <iostream> 
#include <fstream> 
#include <sstream> 
#include <cmath>
#include "std_msgs/msg/float64_multi_array.hpp"

#include "march_shared_msgs/srv/get_current_joint_positions.hpp"

class GaitPlanningAnglesNode:public rclcpp::Node 
{
public: 
    explicit GaitPlanningAnglesNode(); 

private: 
    // std::vector<double> getCurrentJointAngles();

    // Callback for the current exoMode
    void currentModeCallback(const march_shared_msgs::msg::ExoMode::SharedPtr msg);

    // Callback for current joint angles
    void currentJointAnglesCallback(const march_shared_msgs::msg::StateEstimation::SharedPtr msg); 

    // Process various types of gaits 
    void processHomeStandGait();
    void finishGaitBeforeStand(); 
    void processMovingGaits(const int &counter);     
    
    // Final joint angle trajectory publisher
    void publishJointTrajectoryPoints(); 
    std::vector<double> remapKinematicChaintoAlphabetical(const std::vector<double> &kinematic_vector);
    // Member variables 
    rclcpp::Subscription<march_shared_msgs::msg::ExoMode>::SharedPtr m_exo_mode_subscriber; 
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr m_joint_angle_trajectory_publisher; 
    // rclcpp::TimerBase::SharedPtr m_timer;
    // rclcpp::Client<march_shared_msgs::srv::GetCurrentJointPositions>::SharedPtr m_get_current_joint_angles_client;
    rclcpp::Subscription<march_shared_msgs::msg::StateEstimation>::SharedPtr m_current_state_subscriber; 
    
    GaitPlanningAngles m_gait_planning; 
    std_msgs::msg::Float64MultiArray m_joints_msg;
    // trajectory_msgs::msg::JointTrajectoryPoint m_trajectory_prev_point;
    // trajectory_msgs::msg::JointTrajectoryPoint m_trajectory_des_point;

    std::vector<std::vector<double>> m_current_trajectory; 

    std::vector<double> m_incremental_steps_to_home_stand;
    bool m_first_stand;
    std::vector<double> m_initial_point;
    uint8_t m_stance_leg; 
    bool m_single_execution_done; 

};
