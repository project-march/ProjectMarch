//
// Created by rixt on 12-4-23.
//

#ifndef MARCH_FUZZY_DECIDER_HPP
#define MARCH_FUZZY_DECIDER_HPP
#include "geometry_msgs/msg/point_stamped.hpp"
#include "rclcpp/rclcpp.hpp"

class FuzzyDecider {
public:
    FuzzyDecider();

    //getters and setters
    void setPosition(geometry_msgs::msg::PointStamped msg);
    void setTorque(geometry_msgs::msg::PointStamped msg);
    geometry_msgs::msg::PointStamped getPosition();
    geometry_msgs::msg::PointStamped getTorque();

private:
    geometry_msgs::msg::PointStamped m_position;
    geometry_msgs::msg::PointStamped m_torque;
};

#endif //MARCH_FUZZY_DECIDER_HPP
