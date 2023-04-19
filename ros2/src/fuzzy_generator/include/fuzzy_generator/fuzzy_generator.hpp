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

//enum Side {Left, Right, Both, None};
enum Status {Stance, Swing};

struct Leg {
    Status status = Stance;

    double foot_height = 0;

    float torque_weight = 0.5; // holds the weight for the torque
    float position_weight = 0.5; // holds the weight for the position

    // setters
    void setTorqueWeight(float weight){ torque_weight = weight; }; // set the weight for the torque (does not publish the weight yet)
    void setPositionWeight(float weight){ position_weight = weight; }; // set the weight for the position (does not publish the weight yet)

    // getters
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

    Leg* getLeftLeg();
    Leg* getRightLeg();

    // the function that will update the weights with the fuzzy logic
    void updateWeights(Leg leg);


private:
    // above full_position height we use 100% position control
    // below full_torque height we use 100% torque control
    // and linearly decrease/increase in between
    double full_position;
    double full_torque;

    Leg left_leg;
    Leg right_leg;

};

#endif //MARCH_FUZZY_GENERATOR_HPP