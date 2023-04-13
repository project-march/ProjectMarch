//
// Created by rixt on 12-4-23.
//

#ifndef MARCH_FUZZY_GENERATOR_HPP
#define MARCH_FUZZY_GENERATOR_HPP
#include "geometry_msgs/msg/point_stamped.hpp"
#include "march_shared_msgs/msg/feet_height_stamped.hpp"
#include "march_shared_msgs/msg/torque_stamped.hpp"
#include "std_msgs/msg/int32.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"

enum Side {Left, Right, Both, None};

struct Leg {
    geometry_msgs::msg::Point position;
    march_shared_msgs::msg::TorqueStamped torque;
    double foot_height = 0;
    Side side;

    float torque_weight = 0.5; // holds the weight for the torque
    float position_weight = 0.5; // holds the weight for the position

    // setters
    void setPosition(geometry_msgs::msg::Point p){ position = p;}; // set the position of the passed leg
    void setTorque(march_shared_msgs::msg::TorqueStamped t){ torque = t ;}; // set the torque of the passed leg
    void setTorqueWeight(float weight){ torque_weight = weight; }; // set the weight for the torque (does not publish the weight yet)
    void setPositionWeight(float weight){ position_weight = weight; }; // set the weight for the position (does not publish the weight yet)

    // getters
    geometry_msgs::msg::Point getPosition(){ return position; };
    march_shared_msgs::msg::TorqueStamped getTorque(){ return torque; };
    double getFootHeight(){ return foot_height; };
    float getTorqueWeight(){ return torque_weight; };
    float getPositionWeight(){ return position_weight; };
};

class FuzzyGenerator {
public:
    FuzzyGenerator();

    // the function that will update the weights with the fuzzy logic
    void updateWeights(Leg* leg);

    void setFeetHeight(march_shared_msgs::msg::FeetHeightStamped msg); // set the height of both feet
    void setStanceLeg(std_msgs::msg::Int32 msg); // set the stance leg to the correct leg

    Leg* getStanceLeg();
    Leg* getSwingLeg();

    // the function that will update the weights with the fuzzy logic
    void updateWeights(Leg leg);


private:
    double distance_torque; // threshold for at what foot-height to START switching to torque
    double h_offset; // height at which to be COMPLETELY on torque

    Leg left_leg;
    Leg right_leg;

    Side swing_side = None; // which leg is the swing leg
    Side stance_side = Both; // which leg is the stance leg
};

#endif //MARCH_FUZZY_GENERATOR_HPP