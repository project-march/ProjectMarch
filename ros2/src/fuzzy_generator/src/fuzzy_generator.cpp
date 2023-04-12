//
// Created by rixt on 12-4-23.
//

#include "fuzzy_generator/fuzzy_generator.hpp"

FuzzyGenerator::FuzzyGenerator(){

    // initialize both feet to 0
    left_foot.position.x = 0.0;
    left_foot.position.y = 0.0;
    left_foot.position.z = 0.0;
    left_foot.torque.data = 0.0f;

    right_foot.position.x = 0.0;
    right_foot.position.y = 0.0;
    right_foot.position.z = 0.0;
    right_foot.torque.data = 0.0f;
};

void FuzzyGenerator::decideWeights(){
    //TODO: implement fuzzy logic
};

// getters and setters

Foot* FuzzyGenerator::getFoot(Leg leg){
    switch(leg){
        Left:
            return &left_foot;
        Right:
            return &right_foot;
        Both:
            RCLCPP_INFO(rclcpp::get_logger("fuzzy_logger"), "Cannot get both feet");
            break;
        None:
            RCLCPP_INFO(rclcpp::get_logger("fuzzy_logger"), "Cannot get none feet");
            break;
    }
};

void FuzzyGenerator::setFootPosition(geometry_msgs::msg::Point point, Leg leg){
    switch(leg){
        Left:{
            left_foot.position = point;
            break;
        }
        Right:{
            right_foot.position = point;
            break;
        }
        Both:{
            RCLCPP_INFO(rclcpp::get_logger("fuzzy_logger"), "Cannot set both foot positions");
            break;
        }
        None:{
            RCLCPP_INFO(rclcpp::get_logger("fuzzy_logger"), "Cannot set none foot positions");
            break;
        }
    }
};

void FuzzyGenerator::setFootTorque(std_msgs::msg::Float32 torque, Leg leg){
    switch(leg){
        Left:{
            left_foot.torque = torque;
            break;
        }
        Right:{
            right_foot.torque = torque;
            break;
        }
        Both:{
            RCLCPP_INFO(rclcpp::get_logger("fuzzy_logger"), "Cannot set both foot torques");
            break;
        }
        None:{
            RCLCPP_INFO(rclcpp::get_logger("fuzzy_logger"), "Cannot set none foot torques");
            break;
        }
    }
};

void FuzzyGenerator::setFeetHeight(march_shared_msgs::msg::FeetHeightStamped msg){
    left_foot.height = msg.heights[0];
    right_foot.height = msg.heights[1];
};

void FuzzyGenerator::setStanceLeg(std_msgs::msg::Int32 msg){
    switch(msg.data){
        case -1:{
            stance_leg = Left;
            RCLCPP_INFO(rclcpp::get_logger("fuzzy_logger"), "Left foot is on the ground");
            break;
        }
        case 1:{
            stance_leg = Right;
            RCLCPP_INFO(rclcpp::get_logger("fuzzy_logger"), "Right foot is on the ground");
            break;
        }
        case 0:{ //this is the case when both feet are on the ground
            stance_leg = Both;
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

geometry_msgs::msg::Point FuzzyGenerator::getFootPosition(Leg leg){
    switch(leg){
        Left:
            return left_foot.position;
        Right:
            return right_foot.position;
        Both:
            RCLCPP_INFO(rclcpp::get_logger("fuzzy_logger"), "Cannot get both foot positions");
            break;
        None:
            RCLCPP_INFO(rclcpp::get_logger("fuzzy_logger"), "Cannot get none foot positions");
            break;
    }
};
std_msgs::msg::Float32 FuzzyGenerator::getFootTorque(Leg leg){
    switch(leg){
        Left:
            return left_foot.torque;
        Right:
            return right_foot.torque;
        Both:
            RCLCPP_INFO(rclcpp::get_logger("fuzzy_logger"), "Cannot get both foot torques");
            break;
        None:
            RCLCPP_INFO(rclcpp::get_logger("fuzzy_logger"), "Cannot get none foot torques");
            break;
    }};

double FuzzyGenerator::getFootHeight(Leg leg){
    switch(leg){
        Left:
            return left_foot.height;
        Right:
            return right_foot.height;
        Both:
            RCLCPP_INFO(rclcpp::get_logger("fuzzy_logger"), "Cannot get both foot heights");
            break;
        None:
            RCLCPP_INFO(rclcpp::get_logger("fuzzy_logger"), "Cannot get none foot heights");
            break;
    }};

Leg FuzzyGenerator::getStanceLeg(){
    return stance_leg;
};

Leg FuzzyGenerator::getSwingLeg(){
    return swing_leg;
};