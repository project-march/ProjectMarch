//
// Created by rixt on 12-4-23.
//

#include "fuzzy_generator/fuzzy_generator.hpp"

FuzzyGenerator::FuzzyGenerator(){
}

FuzzyGenerator::FuzzyGenerator(std::string config_path){
    config_ = YAML::LoadFile(config_path);
    lower_bound = config_["bounds"]["lower_bound"].as<double>();
    upper_bound = config_["bounds"]["upper_bound"].as<double>();
}

std::vector<std::tuple<std::string, float, float>> FuzzyGenerator::calculateWeights(std::vector<double> both_foot_heights){
    // get the joints, with their torque ranges from the yaml
    const auto torque_ranges = getTorqueRanges();
    std::vector<std::tuple<std::string, float, float>> joints;

    // get stance leg
    both_foot_heights[1] -= both_foot_heights[0];
    both_foot_heights[0] = 0;
    float min = std::min(both_foot_heights[0], both_foot_heights[1]);
    both_foot_heights[0] -= min;
    both_foot_heights[1] -= min;

    // for each joint in the leg, calculate the torque weight and position weight
    for (auto t : torque_ranges) {
        const std::string joint_name = std::get<0>(t);
        const float minimum_torque_percentage = std::get<1>(t);
        const float maximum_torque_percentage = std::get<2>(t);
        const float torque_range = maximum_torque_percentage - minimum_torque_percentage;

        // getting the correct foot height and leg
        float foot_height;
        std::string current_leg;

        if (joint_name.find("left") != std::string::npos) {
            foot_height = both_foot_heights[0];
            current_leg = "left";
        } else if (joint_name.find("right") != std::string::npos) {
            foot_height = both_foot_heights[1];
            current_leg = "right";
        } else {
            // if the joint name does not contain 'left' or 'right', it is not a leg joint > check with girls
            throw std::runtime_error("Could not determine if joint " + joint_name + " was left or right.");
        }

        // calculate how far the foot is in the 'fuzzy shifting range'
        const float height_percentage = (upper_bound - foot_height) / (upper_bound - lower_bound);
        float torque_weight = torque_range * height_percentage;
        torque_weight = std::max(minimum_torque_percentage, std::min(torque_weight, maximum_torque_percentage));
        const float position_weight = 1 - torque_weight;

        joints.push_back(std::make_tuple(joint_name, position_weight, torque_weight));
    }
    return joints;
}

std::vector<std::tuple<std::string, float, float>> FuzzyGenerator::getTorqueRanges()
{
    std::vector<std::tuple<std::string, /*position_weight=*/float, /*torque_weight=*/float>> joints;
    YAML::Node joints_config = config_["joints"];

    for (YAML::const_iterator it = joints_config.begin(); it != joints_config.end(); ++it) {
        std::string joint_name = it->first.as<std::string>();

        float min_torque = joints_config[joint_name]["minimum_torque"].as<float>();
        float max_torque = joints_config[joint_name]["maximum_torque"].as<float>();

        joints.push_back(std::make_tuple(joint_name, min_torque, max_torque));
    }
    return joints;
}