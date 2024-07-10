#ifndef FOOTSTEP_GEN_H
#define FOOTSTEP_GEN_H

#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "march_shared_msgs/msg/exo_mode.hpp"
#include "march_shared_msgs/srv/request_footsteps.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include <chrono>
#include <cstdio>

class FootstepGenerator : public rclcpp::Node {
public:
    FootstepGenerator();

    geometry_msgs::msg::PoseArray generate_foot_placements(int stance_leg, int gait_type);

    int get_steps();
    double get_velocity_x();
    double get_velocity_y();
    double get_feet_spread();

private:
    void exo_mode_callback(const march_shared_msgs::msg::ExoMode::SharedPtr msg);
    void publish_foot_placements(const std::shared_ptr<march_shared_msgs::srv::RequestFootsteps::Request> request,
        std::shared_ptr<march_shared_msgs::srv::RequestFootsteps::Response> response);

    void publish_footsteps(geometry_msgs::msg::PoseArray footsteps);

    rclcpp::Subscription<march_shared_msgs::msg::ExoMode>::SharedPtr m_exo_mode_sub;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr m_publisher;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr m_swing_trajectory_command_publisher;
    rclcpp::Service<march_shared_msgs::srv::RequestFootsteps>::SharedPtr m_service;

    int m_steps;33
    double m_vx;
    double m_vy;
    double m_l;
};

#endif
