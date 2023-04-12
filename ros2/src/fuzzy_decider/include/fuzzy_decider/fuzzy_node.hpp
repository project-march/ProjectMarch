//
// Created by rixt on 12-4-23.
//

#ifndef MARCH_FUZZY_NODE_HPP
#define MARCH_FUZZY_NODE_HPP
#include "rclcpp/rclcpp.hpp"
#include "fuzzy_decider.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"

class FuzzyNode : public rclcpp::Node {
public:
    FuzzyNode();

private:
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr m_position_subscription;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr m_torque_subscription;

    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr m_publish_pos_weight;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr m_publish_torque_weight;

    void position_callback(geometry_msgs::msg::PointStamped::SharedPtr msg);
    void torque_callback(geometry_msgs::msg::PointStamped::SharedPtr msg);

    FuzzyDecider m_fuzzy_decider;
};
#endif //MARCH_FUZZY_NODE_HPP
