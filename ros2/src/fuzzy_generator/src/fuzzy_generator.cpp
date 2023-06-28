//
// Created by rixt on 12-4-23.
//

#include "fuzzy_generator/fuzzy_generator.hpp"

FuzzyGenerator::FuzzyGenerator()
{

    //TODO: implement
    // auto yaml_location = 'joints.yaml'

    // YAML::Node config_ = YAML::LoadFile("joints.yaml");
    // const auto bounds = config_.begin()->first.as<std::string>();
    
    // lower_bound = config["range"]["lower_bound"].as<double>();
    // upper_bound = config["range"]["upper_bound"].as<double>();
};


std::vector<std::tuple<std::string, float, float>> FuzzyGenerator::calculateWeights(std::string leg, float foot_height){
    
    // calculate far the foot is in the 'fuzzy shifting range'
    float total_range = upper_bound - lower_bound;
    float actual_range = upper_bound - foot_height;
    float torque_percentage = actual_range/total_range;

    // get the joints, with their torque ranges from the yaml
    std::vector<std::tuple<std::string, float, float>> torque_ranges = getTorqueRanges(leg);
    std::vector<std::tuple<std::string, float, float>>  joints;

    // for each joint in the leg, calculate the torque weight and position weight
    for(auto t: torque_ranges){
        float torque_range = std::get<2>(t) - std::get<1>(t);

        float torque_weight = torque_range * torque_percentage;
        float position_weight = 1 - torque_weight;

        joints.push_back(std::make_tuple(std::get<0>(t), position_weight, torque_weight));
    }

    return joints;
}

std::vector<std::tuple<std::string, float, float>>  FuzzyGenerator::getTorqueRanges(std::string leg){
    //TODO: implement
    std::vector<std::tuple<std::string, /*position_weight=*/float, /*torque_weight=*/float>> joints;

    // const auto joint_name = this->robot_config_.begin()->first.as<std::string>();

    // YAML::Node lineup = YAML::Load("{1B: Prince Fielder, 2B: Rickie Weeks, LF: Ryan Braun}");

    // YAML::Node joint_config = config["joints"];

    // for(YAML::const_iterator it=joint_config.begin();it!=joint_config.end();++it) {
    //     std::string joint_name = it->first.as<std::string>();

    //     // std::cout << "Playing at " << it->first.as<std::string>() << " is " << it->second.as<std::string>() << "\n";
    // }
    return joints;
}

float FuzzyGenerator::getUpperBound(){
    return upper_bound;
}

float FuzzyGenerator::getLowerBound(){
    return lower_bound;
}