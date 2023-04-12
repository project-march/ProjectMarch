//
// Created by rixt on 12-4-23.
//

#ifndef MARCH_FUZZY_GENERATOR_HPP
#define MARCH_FUZZY_GENERATOR_HPP
#include "geometry_msgs/msg/point_stamped.hpp"
#include "march_shared_msgs/msg/feet_height_stamped.hpp"
#include "std_msgs/msg/int32.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"


struct Foot {
    char foot;
    geometry_msgs::msg::Point position;
    std_msgs::msg::Float32 torque;
    double height;
};

enum Leg {Left, Right, Both, None};

class FuzzyGenerator {
public:
    FuzzyGenerator();

    void decideWeights();

    //getters and setters
    void setFootPosition(geometry_msgs::msg::Point msg, Leg leg);
    void setFootTorque(std_msgs::msg::Float32 msg, Leg leg);
    void setFeetHeight(march_shared_msgs::msg::FeetHeightStamped msg);
    void setStanceLeg(std_msgs::msg::Int32 msg);
    geometry_msgs::msg::Point getFootPosition(Leg leg);
    std_msgs::msg::Float32 getFootTorque(Leg leg);
    double getFootHeight(Leg leg);
    Leg getStanceLeg();
    Leg getSwingLeg();
    Foot* getFoot(Leg leg);

private:
    double distance_torque; // threshold for at what foot-height to START switching to torque
    double h_offset; // height at which to be COMPLETELY on torque

    Foot* left_foot;
    Foot* right_foot;

    Leg swing_leg;
    Leg stance_leg;
};

#endif //MARCH_FUZZY_GENERATOR_HPP