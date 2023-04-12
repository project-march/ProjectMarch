//
// Created by rixt on 12-4-23.
//

#ifndef MARCH_FUZZY_DECIDER_HPP
#define MARCH_FUZZY_DECIDER_HPP
#include "geometry_msgs/msg/point_stamped.hpp"
#include "rclcpp/rclcpp.hpp"

struct Foot {
    char foot;
    geometry_msgs::msg::Point position;
    geometry_msgs::msg::Point torque;
    double height;
};

class FuzzyDecider {
public:
    FuzzyDecider();

    void decideWeights();

    //getters and setters
    void setPosition(geometry_msgs::msg::Point msg);
    void setTorque(geometry_msgs::msg::Point msg);
    geometry_msgs::msg::Point getPosition();
    geometry_msgs::msg::Point getTorque();
    Foot* getFoot(const char* prefix);

private:
    geometry_msgs::msg::Point m_position;
    geometry_msgs::msg::Point m_torque;
    double distance_torque; // threshold for at what foot-height to START switching to torque
    double h_offset; // height at which to be COMPLETELY on torque

    Foot* left_foot;
    Foot* right_foot;

    char* swing_leg; // 'l' if left leg is current stance leg, otherwise 'r'
    char* stance_leg; // 'l' if left leg is current stance leg, otherwise 'r'
};

#endif //MARCH_FUZZY_DECIDER_HPP
