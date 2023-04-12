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
#include "std_msgs/msg/float32.hpp"

class FuzzyNode : public rclcpp::Node {
public:
    FuzzyNode();

private:
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr m_position_subscription;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr m_torque_subscription;
    rclcpp::Subscription<march_shared_msgs::msg::FeetHeightStamped>::SharedPtr m_foot_height_subscription;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr m_stance_leg_subscription;

    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr m_publish_pos_weight;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr m_publish_torque_weight;
//
    void position_callback(geometry_msgs::msg::PointStamped::SharedPtr msg);
    void torque_callback(std_msgs::msg::Float32::SharedPtr msg);
    void stance_leg_callback(std_msgs::msg::Int32::SharedPtr msg);
    void height_callback(march_shared_msgs::msg::FeetHeightStamped::SharedPtr msg);

    FuzzyGenerator m_fuzzy_generator;
};
#endif //MARCH_FUZZY_NODE_HPP