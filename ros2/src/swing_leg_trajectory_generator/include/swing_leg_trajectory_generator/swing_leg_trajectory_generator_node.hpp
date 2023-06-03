//
// Created by Marco Bak M8 on 8-3-23.
//

#ifndef BUILD_SWING_LEG_TRAJECTORY_GENERATOR_NODE_HPP
#define BUILD_SWING_LEG_TRAJECTORY_GENERATOR_NODE_HPP
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "march_shared_msgs/msg/center_of_mass.hpp"
#include "march_shared_msgs/msg/gait_type.hpp"
#include "march_shared_msgs/srv/gait_command.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include "swing_leg_trajectory_generator/swing_leg_trajectory_generator.hpp"
#include <chrono>
#include <cstdio>
#include <string>

class SwingLegTrajectoryGeneratorNode : public rclcpp::Node {
public:
    SwingLegTrajectoryGeneratorNode();

private:
    //    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr m_publish_curve;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr m_publish_curve;
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr m_points_subscription;
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr m_final_feet_subscriber;
    // rclcpp::Subscription<march_shared_msgs::msg::CenterOfMass>::SharedPtr m_com_subscriber;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr m_stance_feet_subscriber;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr m_weight_shift_subscriber;

    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr m_path_publisher;

    void subscriber_callback(geometry_msgs::msg::PoseArray::SharedPtr msg);
    void weight_shift_callback(std_msgs::msg::Int32::SharedPtr msg);
    void stance_feet_callback(std_msgs::msg::Int32::SharedPtr msg);
    void final_feet_callback(geometry_msgs::msg::PoseArray::SharedPtr msg);
    void com_callback(march_shared_msgs::msg::CenterOfMass::SharedPtr msg);
    void publish_zero_swing();

    void publish_path_visualization();
    SwingLegTrajectoryGenerator m_swing_leg_generator;

    double prev_step_size;
};
#endif // BUILD_SWING_LEG_TRAJECTORY_GENERATOR_NODE_HPP
