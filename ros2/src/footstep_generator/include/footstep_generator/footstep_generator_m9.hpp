#ifndef FOOTSTEP_GEN_H
#define FOOTSTEP_GEN_H

#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "march_shared_msgs/srv/request_footsteps.hpp"
#include "march_shared_msgs/msg/state_estimation.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include <chrono>
#include <cstdio>

class FootstepGenerator : public rclcpp::Node {
public:

    FootstepGenerator();

    geometry_msgs::msg::PoseArray generateFootPlacements(int gait_type);

    int getSteps();
    double getVelocityX();
    double getVelocityY();
    double getStepLength();

private:
    void publishFootPlacements(const std::shared_ptr<march_shared_msgs::srv::RequestFootsteps::Request> request,
        std::shared_ptr<march_shared_msgs::srv::RequestFootsteps::Response> response);
    void publishFootsteps(geometry_msgs::msg::PoseArray footsteps);
    void currentExoJointStateCallback(const march_shared_msgs::msg::StateEstimation::SharedPtr msg); 

    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr m_footstep_publisher;
    // rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr m_swing_trajectory_command_publisher;
    rclcpp::Service<march_shared_msgs::srv::RequestFootsteps>::SharedPtr m_request_footsteps_service;
    rclcpp::Subscription<march_shared_msgs::msg::StateEstimation>::SharedPtr m_exo_joint_state_subscriber; 

    int m_steps;
    double m_velocity_x;
    double m_velocity_y;
    double m_step_length;
    
    geometry_msgs::msg::Pose m_current_left_foot; 
    geometry_msgs::msg::Pose m_current_right_foot; 
};

#endif
