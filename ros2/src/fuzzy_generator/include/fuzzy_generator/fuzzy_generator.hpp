//
// Created by rixt on 12-4-23.
//

#ifndef MARCH_FUZZY_GENERATOR_HPP
#define MARCH_FUZZY_GENERATOR_HPP
#include "geometry_msgs/msg/point_stamped.hpp"
#include "march_shared_msgs/msg/feet_height_stamped.hpp"
#include "std_msgs/msg/int32.hpp"
#include "rclcpp/rclcpp.hpp"

struct Foot {
    char foot;
    geometry_msgs::msg::Point position;
    geometry_msgs::msg::Point torque;
    double height;
};

class FuzzyGenerator {
public:
    FuzzyGenerator();

    void decideWeights();

    //getters and setters
    void setFootPosition(geometry_msgs::msg::Point msg, const char* prefix);
    void setFootTorque(geometry_msgs::msg::Point msg, const char* prefix);
    void setFootHeight(march_shared_msgs::msg::FeetHeightStamped msg, const char* prefix);
    void setStanceLeg(std_msgs::msg::Int32 msg);
    geometry_msgs::msg::Point getFootPosition(const char* prefix);
    geometry_msgs::msg::Point getFootTorque(const char* prefix);
    double getFootHeight(const char* prefix);
    char* getStanceLeg();
    Foot* getFoot(const char* prefix);

private:
    double distance_torque; // threshold for at what foot-height to START switching to torque
    double h_offset; // height at which to be COMPLETELY on torque

    Foot* left_foot;
    Foot* right_foot;

//    char* swing_leg; // 'l' if left leg is current stance leg, otherwise 'r'
    char* stance_leg; // 'l' if left leg is current stance leg, 'r' if right leg is current stance leg
    bool double_stance;
};

#endif //MARCH_FUZZY_GENERATOR_HPP