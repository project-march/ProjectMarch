//
// Created by Marco Bak M8 on 8-3-23.
//

#ifndef BUILD_SWING_LEG_TRAJECTORY_GENERATOR_NODE_HPP
#define BUILD_SWING_LEG_TRAJECTORY_GENERATOR_NODE_HPP
#include "geometry_msgs/msg/point_stamped.hpp"
#include "march_shared_msgs/msg/gait_type.hpp"
#include "march_shared_msgs/msg/point_stamped_list.hpp"
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
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr m_publish_curve;
    rclcpp::Subscription<march_shared_msgs::msg::PointStampedList>::SharedPtr m_points_subscription;

    void subscriber_callback(march_shared_msgs::msg::PointStampedList::SharedPtr msg);
    rclcpp::TimerBase::SharedPtr timer_;

    SwingLegTrajectoryGenerator m_swing_leg_generator;
};
#endif // BUILD_SWING_LEG_TRAJECTORY_GENERATOR_NODE_HPP
