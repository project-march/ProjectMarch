/*Authors: Femke Buiks and Andrew Hutani, MIX*/

#pragma once 

#include <string>
#include <vector>
#include <array>  
#include <set>
#include "rclcpp/rclcpp.hpp" 
#include "march_gait_planning/gait_planning.hpp"
#include "march_shared_msgs/msg/iks_foot_positions.hpp"
#include "march_shared_msgs/msg/state_estimation.hpp"
#include "march_shared_msgs/msg/exo_mode.hpp"
#include "march_shared_msgs/msg/foot_step_output.hpp"
#include "std_msgs/msg/int32.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "march_shared_msgs/srv/get_current_stance_leg.hpp"


class GaitPlanningNode:public rclcpp::Node {
    public: 
    explicit GaitPlanningNode();
    // typedef std::array<double, 4> XZFeetPositionsArray; 
    // typedef std::array<double, 3> XYZFootPositionArray; 

    private: 

    //Temporary struct to help remove duplicate values from the final_feet_position message (7 digits after decimal)
    struct PoseXComparator {
        bool operator()(const geometry_msgs::msg::Pose& pose1, const geometry_msgs::msg::Pose& pose2) const {
            // return pose1.position.x < pose2.position.x;

            double rounded_x1 = round(pose1.position.x * 1e7) / 1e7;
            double rounded_x2 = round(pose2.position.x * 1e7) / 1e7;
            return rounded_x1 < rounded_x2;
        }
    };
    
    rclcpp::Publisher<march_shared_msgs::msg::IksFootPositions>::SharedPtr m_iks_foot_positions_publisher; 
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr m_interpolated_bezier_visualization_publisher; 
    rclcpp::Subscription<march_shared_msgs::msg::ExoMode>::SharedPtr m_exo_mode_subscriber; // exo_mode == gait_type 
    rclcpp::Subscription<march_shared_msgs::msg::StateEstimation>::SharedPtr m_exo_joint_state_subscriber; 

    //Create subscription to the output of the footstep planner, aka the distance of the next step. 
    // rclcpp::Subscription<march_shared_msgs::msg::FootStepOutput>::SharedPtr m_variable_foot_step_subscriber; 
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr m_mpc_foot_positions_subscriber; 

    void currentModeCallback(const march_shared_msgs::msg::ExoMode::SharedPtr msg); 
    void currentExoJointStateCallback(const march_shared_msgs::msg::StateEstimation::SharedPtr msg); 
    void MPCCallback(const geometry_msgs::msg::PoseArray::SharedPtr msg); 
    void setFootPositionsMessage(double left_x, double left_y, double left_z, 
                            double right_x, double right_y, double right_z);
    void footPositionsPublish(); 
    void processStand(); 
    void finishCurrentTrajectory(); 
    void publishIncrements(); 
    void stepClose(); 
    void calculateIncrements(); 
    void publishHomeStand(); 
    void publishWalk(); 
    void publishHeightGaits(); 
    void publishVariableWalk(); 

    GaitPlanning m_gait_planning; 

    GaitPlanning::XYZFootPositionArray m_left_foot_offset; 
    GaitPlanning::XYZFootPositionArray m_right_foot_offset; 
    std::vector<std::array<double, 6>> m_home_stand_trajectory; 
    std::array<double, 6> m_initial_position; 
    std::vector<double> m_increments; 

    std::vector<GaitPlanning::XZFeetPositionsArray> m_current_trajectory; 
    march_shared_msgs::msg::IksFootPositions::SharedPtr m_desired_footpositions_msg;
    geometry_msgs::msg::Pose::SharedPtr m_pose;  
    geometry_msgs::msg::PoseArray::SharedPtr m_visualization_msg; 
    rclcpp::TimerBase::SharedPtr m_timer;
    std::vector<double> m_home_stand; 
    bool m_single_execution_done;
    bool m_variable_first_step_done; 
    int m_variable_walk_swing_leg; 

};