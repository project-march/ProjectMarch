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
    void setPosition(geometry_msgs::msg::Point msg);
    void setTorque(geometry_msgs::msg::Point msg);
    geometry_msgs::msg::Point getPosition();
    geometry_msgs::msg::Point getTorque();

private:
    geometry_msgs::msg::Point m_position;
    geometry_msgs::msg::Point m_torque;
};

#endif //MARCH_FUZZY_DECIDER_HPP
