//
// Created by rixt on 12-4-23.
//

#include "fuzzy_generator/fuzzy_generator.hpp"

FuzzyGenerator::FuzzyGenerator(){
};

void FuzzyGenerator::updateWeights(Leg* leg){
    //TODO: differentiate between logic for stance -> swing and swing -> stance
    leg->position_weight = (1 / full_torque) * leg->getFootHeight() - full_position;
    leg->torque_weight = 1 - leg->position_weight;
};

// setters

void FuzzyGenerator::setFeetHeight(march_shared_msgs::msg::FeetHeightStamped msg){
    left_leg.foot_height = msg.heights[0];
    right_leg.foot_height = msg.heights[1];
};

void FuzzyGenerator::setStanceLeg(std_msgs::msg::Int32 msg){
    switch(msg.data){
        case -1:{
            left_leg.status = Stance;
            right_leg.status = Swing;
            RCLCPP_INFO(rclcpp::get_logger("fuzzy_logger"), "Left foot is on the ground, Right foot is up");
            break;
        }
        case 1:{
            left_leg.status = Swing;
            right_leg.status = Stance;
            RCLCPP_INFO(rclcpp::get_logger("fuzzy_logger"), "Right foot is on the ground, Left foot is up");
            break;
        }
        case 0:{ //this is the case when both feet are on the ground
            left_leg.status = Stance;
            right_leg.status = Stance;
            RCLCPP_INFO(rclcpp::get_logger("fuzzy_logger"), "Both feet are on the ground");
            break;
        }
        default:{
            RCLCPP_INFO(rclcpp::get_logger("fuzzy_logger"), "Invalid value %i was passed. Allowed values are -1,0,1", msg.data);
        }
    }
}

// getters

Leg* FuzzyGenerator::getLeftLeg(){
    return &left_leg;
}

Leg* FuzzyGenerator::getRightLeg(){
    return &right_leg;
}