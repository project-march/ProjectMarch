//
// Created by rixt on 12-4-23.
//

#include "fuzzy_generator/fuzzy_generator.hpp"

FuzzyGenerator::FuzzyGenerator(){

    // initialize both feet to 0
    left_leg.position.x = 0.0;
    left_leg.position.y = 0.0;
    left_leg.position.z = 0.0;
    left_leg.torque.data = 0.0f;

    right_leg.position.x = 0.0;
    right_leg.position.y = 0.0;
    right_leg.position.z = 0.0;
    right_leg.torque.data = 0.0f;
};

void FuzzyGenerator::updateWeights(Leg* leg){
    //TODO(Sofie): implement fuzzy logic
};

// setters

void FuzzyGenerator::setFeetHeight(march_shared_msgs::msg::FeetHeightStamped msg){
    left_leg.foot_height = msg.heights[0];
    right_leg.foot_height = msg.heights[1];
};

void FuzzyGenerator::setStanceLeg(std_msgs::msg::Int32 msg){
    switch(msg.data){
        case -1:{
            stance_side = Left;
            swing_side = Right;
            RCLCPP_INFO(rclcpp::get_logger("fuzzy_logger"), "Left foot is on the ground, Right foot is up");
            break;
        }
        case 1:{
            stance_side = Right;
            swing_side = Left;
            RCLCPP_INFO(rclcpp::get_logger("fuzzy_logger"), "Right foot is on the ground, Left foot is up");
            break;
        }
        case 0:{ //this is the case when both feet are on the ground
            stance_side = Both;
            swing_side = None;
            RCLCPP_INFO(rclcpp::get_logger("fuzzy_logger"), "Both feet are on the ground");
            break;
        }
        default:{
            RCLCPP_INFO(rclcpp::get_logger("fuzzy_logger"), "Invalid value %i was passed. Allowed values are -1,0,1", msg.data);
        }
    }
//    if(prefix != 'l' and prefix != 'r' and !m_fuzzy_generator->double_stance){
//        RCLCPP_WARN(this->get_logger(), "Method getFoot was passed invalid initial character %s. Required: 'l' or 'r'", prefix);
//    }
}

// getters

Leg* FuzzyGenerator::getStanceLeg(){
    Side side = stance_side;
    switch(side){
        Left:
            return &left_leg;
        Right:
            return &right_leg;
        Both:
            RCLCPP_INFO(rclcpp::get_logger("fuzzy_logger"), "Both legs are stance legs");
            break;
        None:
            RCLCPP_INFO(rclcpp::get_logger("fuzzy_logger"), " Neither legs are stance legs");
            break;
    }};

Leg* FuzzyGenerator::getSwingLeg(){
    Side side = swing_side;
    switch(side){
        Left:
            return &left_leg;
        Right:
            return &right_leg;
        Both:
            RCLCPP_INFO(rclcpp::get_logger("fuzzy_logger"), "Both legs are swing legs");
            break;
        None:
            RCLCPP_INFO(rclcpp::get_logger("fuzzy_logger"), " Neither legs are swing legs");
            break;
    }
};