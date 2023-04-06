//
// Created by Marco Bak M8 on 8-3-23.
//

#ifndef BUILD_SWING_LEG_TRAJECTORY_GENERATOR_NODE_HPP
#define BUILD_SWING_LEG_TRAJECTORY_GENERATOR_NODE_HPP
#include "geometry_msgs/msg/point_stamped.hpp"
#include "march_shared_msgs/msg/gait_type.hpp"
//#include "march_shared_msgs/msg/point_stamped_list.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "march_shared_msgs/srv/gait_command.hpp"
#include "rclcpp/rclcpp.hpp"
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

    void subscriber_callback(geometry_msgs::msg::PoseArray::SharedPtr msg);
    void final_feet_callback(geometry_msgs::msg::PoseArray::SharedPtr msg);

    SwingLegTrajectoryGenerator m_swing_leg_generator;
};
#endif // BUILD_SWING_LEG_TRAJECTORY_GENERATOR_NODE_HPP
