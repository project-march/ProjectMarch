#ifndef FOOTSTEP_GEN_H
#define FOOTSTEP_GEN_H

#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

#include "march_shared_msgs/srv/request_footsteps.hpp"
#include "rclcpp/rclcpp.hpp"
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
    void imu_state_callback(sensor_msgs::msg::Imu::SharedPtr);
    void joint_state_callback(sensor_msgs::msg::JointState::SharedPtr);

    void publish_foot_placements(const std::shared_ptr<march_shared_msgs::srv::RequestFootsteps::Request> request,
        std::shared_ptr<march_shared_msgs::srv::RequestFootsteps::Response> response);

    void publish_footsteps(geometry_msgs::msg::PoseArray footsteps);

    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr m_publisher;
    rclcpp::Service<march_shared_msgs::srv::RequestFootsteps>::SharedPtr m_service;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr m_subscriber_joints;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr m_subscriber_imu;

    int m_steps;
    const double m_vx;
    const double m_vy;
    double m_l;
};

#endif