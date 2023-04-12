//
// Created by rixt on 12-4-23.
//

#ifndef MARCH_FUZZY_NODE_HPP
#define MARCH_FUZZY_NODE_HPP
#include "rclcpp/rclcpp.hpp"
#include "fuzzy_generator.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "march_shared_msgs/msg/feet_height_stamped.hpp"
#include "std_msgs/msg/int32.hpp"

class FuzzyNode : public rclcpp::Node {
public:
    FuzzyNode();

private:
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr m_position_subscription;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr m_torque_subscription;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr m_foot_height_subscription;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr m_swing_leg_subscription;

    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr m_publish_pos_weight;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr m_publish_torque_weight;
//
    void position_callback(geometry_msgs::msg::PointStamped::SharedPtr msg);
    void torque_callback(geometry_msgs::msg::PointStamped::SharedPtr msg);
    void swing_leg_callback(std_msgs::msg::Int32::SharedPtr msg);
//    void height_callback(march_shared_msgs::msg::FeetHeightStamped msg);

    FuzzyGenerator m_fuzzy_generator;
};
#endif //MARCH_FUZZY_NODE_HPP