
#include "rclcpp/rclcpp.hpp" 
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"
#include "march_gait_planning/gait_planning_joint_angles.hpp"
#include "march_shared_msgs/msg/exo_state.hpp"
#include <vector>
#include <array> 
#include <iostream> 
#include <fstream> 
#include <sstream> 
#include <cmath>


class GaitPlanningAnglesNode:public rclcpp::Node {
    
    public: 
    explicit GaitPlanningAnglesNode(); 

    private: 

    void currentStateCallback(const march_shared_msgs::msg::ExoState::SharedPtr msg);
    void initializeConstantsPoints(trajectory_msgs::msg::JointTrajectoryPoint &point); 
    void processContinuousWalkingGait(trajectory_msgs::msg::JointTrajectoryPoint &prev_point, trajectory_msgs::msg::JointTrajectoryPoint &des_point, trajectory_msgs::msg::JointTrajectory &message);
    void processFirstStepWalkingGait(trajectory_msgs::msg::JointTrajectoryPoint &prev_point, trajectory_msgs::msg::JointTrajectoryPoint &des_point, trajectory_msgs::msg::JointTrajectory &message); 
    void processHomeStandGait(trajectory_msgs::msg::JointTrajectoryPoint &prev_point, trajectory_msgs::msg::JointTrajectoryPoint &des_point, trajectory_msgs::msg::JointTrajectory &message);
    void processStandToSitGait(trajectory_msgs::msg::JointTrajectoryPoint &prev_point, trajectory_msgs::msg::JointTrajectoryPoint &des_point, trajectory_msgs::msg::JointTrajectory &message); 
    void publishJointTrajectoryPoints(); 

    rclcpp::Subscription<march_shared_msgs::msg::ExoState>::SharedPtr m_exo_state_subscriber; 
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr m_joint_angle_trajectory_publisher; 
    rclcpp::TimerBase::SharedPtr m_timer;

    GaitPlanningAngles m_gait_planning; 

};
