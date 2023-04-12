//
// Created by rixt on 12-4-23.
//

#include "fuzzy_generator/fuzzy_generator.hpp"

FuzzyGenerator::FuzzyGenerator(){
    //TODO: implement initialization
};

void FuzzyGenerator::decideWeights(){
    //TODO: implement fuzzy logic
};

// getters and setters

Foot* FuzzyGenerator::getFoot(const char* prefix){
    switch(*prefix){
        case 'l':
            return left_foot;
        case 'r':
            return right_foot;
        default:
            return 0;
    }
};


void FuzzyGenerator::setFootPosition(geometry_msgs::msg::Point point, const char* prefix){
    switch(*prefix){
        case * "l":{
            left_foot->position = point;
            break;
        }
        case * "r":{
            right_foot->position = point;
            break;
        }
    }
};
void FuzzyGenerator::setFootTorque(geometry_msgs::msg::Point point, const char* prefix){
    switch(*prefix){
        case * "l":{
            left_foot->torque = point;
            break;
        }
        case * "r":{
            right_foot->torque = point;
            break;
        }
    }
};
void FuzzyGenerator::setFootHeight(march_shared_msgs::msg::FeetHeightStamped h, const char* prefix){
    double height = 0.0;
    switch(*prefix){
        case * "l":{
            left_foot->height = height;
            break;
        }
        case * "r":{
            right_foot->height = height;
            break;
        }

    }
};
void FuzzyGenerator::setStanceLeg(std_msgs::msg::Int32 msg){
    switch(msg.data){
        case -1:{
            stance_leg = "l";
            break;
        }
        case 1:{
            stance_leg = "r";
            break;
        }
        case 0:{
            double_stance = true;
            break;
        }
    }
//    if(prefix != 'l' and prefix != 'r' and !m_fuzzy_generator->double_stance){
//        RCLCPP_WARN(this->get_logger(), "Method getFoot was passed invalid initial character %s. Required: 'l' or 'r'", prefix);
//    }
}

geometry_msgs::msg::Point FuzzyGenerator::getFootPosition(const char* prefix){
    switch(*prefix){
        case * "l":
            return left_foot->position;
        case * "r":
            return right_foot->position;
    }
};
geometry_msgs::msg::Point FuzzyGenerator::getFootTorque(const char* prefix){
    switch(*prefix){
        case * "l":
            return left_foot->torque;
        case * "r":
            return right_foot->torque;
    }};
double FuzzyGenerator::getFootHeight(const char* prefix){
    switch(*prefix){
        case * "l":
            return left_foot->height;
        case * "r":
            return right_foot->height;
    }};
char* FuzzyGenerator::getStanceLeg(){
    return stance_leg;
};