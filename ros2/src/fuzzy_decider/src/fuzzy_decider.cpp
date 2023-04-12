//
// Created by rixt on 12-4-23.
//

#include "fuzzy_decider/fuzzy_decider.hpp"

FuzzyDecider::FuzzyDecider(){
    //TODO: implement
};


void FuzzyDecider::setPosition(geometry_msgs::msg::Point point){
    m_position = point;
};

void FuzzyDecider::setTorque(geometry_msgs::msg::Point point){
    m_torque = point;
};

geometry_msgs::msg::Point FuzzyDecider::getPosition(){
    return m_position;
};
geometry_msgs::msg::Point FuzzyDecider::getTorque(){
    return m_torque;
};