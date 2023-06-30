//
// Created by rixt on 12-4-23.
//

#include "fuzzy_generator/fuzzy_generator.hpp"
#include "rclcpp/rclcpp.hpp"


FuzzyGenerator::FuzzyGenerator()
{

    std::string config_file_path = ament_index_cpp::get_package_share_directory("fuzzy_generator")
    + PATH_SEPARATOR + "config" + PATH_SEPARATOR + "test_joints.yaml";

    config_ = YAML::LoadFile(config_file_path);
    
    lower_bound = config_["bounds"]["lower_bound"].as<double>();
    upper_bound = config_["bounds"]["upper_bound"].as<double>();
};


std::vector<std::tuple<std::string, float, float>> FuzzyGenerator::calculateWeights(std::vector<double> both_foot_heights){
    
    // get the joints, with their torque ranges from the yaml
    std::vector<std::tuple<std::string, float, float>> torque_ranges = getTorqueRanges();
    std::vector<std::tuple<std::string, float, float>> joints;


    // for each joint in the leg, calculate the torque weight and position weight
    for(auto t: torque_ranges){
        std::string joint_name = std::get<0>(t);
        float minimum_torque = std::get<1>(t);
        float maximum_torque = std::get<2>(t);

        // getting the correct foot height
        float foot_height;
        if(joint_name.find("left") != std::string::npos){
            foot_height = both_foot_heights[0];
        }
        else if(joint_name.find("right") != std::string::npos){
            foot_height = both_foot_heights[1];
        }
        else{
            foot_height = both_foot_heights[0];
            RCLCPP_INFO_STREAM(rclcpp::get_logger("fuzzy_generator"), "We could not determine if joint " << joint_name << " was left or right. Assuming test joint and using left foot height");
        }

        // calculate far the foot is in the 'fuzzy shifting range'
        float total_range = upper_bound - lower_bound;
        float actual_range = upper_bound - foot_height;
        float torque_percentage = actual_range/total_range;

        if(foot_height > upper_bound){
            RCLCPP_INFO_STREAM(rclcpp::get_logger("fuzzy_generator"), "joint " << joint_name << " is above range");
            // we should be in 100% position control
            torque_percentage = 0.0f;
        }
        if(foot_height < lower_bound){
            RCLCPP_INFO_STREAM(rclcpp::get_logger("fuzzy_generator"), "joint " << joint_name << " is above range");
            // we should be in as much torque control as is allowed
            torque_percentage = 1.0f;
        }

        // calculating the weights for both position and torque
        float torque_range = maximum_torque - minimum_torque;

        float torque_weight = torque_range * torque_percentage;
        float position_weight = 1 - torque_weight;

        joints.push_back(std::make_tuple(joint_name, position_weight, torque_weight));
        // RCLCPP_INFO_STREAM(rclcpp::get_logger("fuzzy_generator"), "name: " << std::get<1>(t) << " pos weight: " << position_weight << " torque weight: " << torque_weight);
    }

    return joints;
}

std::vector<std::tuple<std::string, float, float>>  FuzzyGenerator::getTorqueRanges(){
    std::vector<std::tuple<std::string, /*position_weight=*/float, /*torque_weight=*/float>> joints;

    YAML::Node joints_config = config_["joints"];
    // RCLCPP_INFO_STREAM(rclcpp::get_logger("fuzzy_generator"), "we got " << leg << " joints:");

    for(YAML::const_iterator it=joints_config.begin();it!=joints_config.end();++it) {
        std::string joint_name = it->first.as<std::string>();
        RCLCPP_INFO_STREAM(rclcpp::get_logger("fuzzy_generator"), "" << joint_name);

        float min_torque = joints_config[joint_name]["minimum_torque"].as<float>();
        float max_torque = joints_config[joint_name]["maximum_torque"].as<float>();
        RCLCPP_INFO_STREAM(rclcpp::get_logger("fuzzy_generator"), "with min torque " << min_torque);
        RCLCPP_INFO_STREAM(rclcpp::get_logger("fuzzy_generator"), "with max torque " << max_torque);

        joints.push_back(std::make_tuple(joint_name, min_torque, max_torque));
    }
    return joints;
}

float FuzzyGenerator::getUpperBound(){
    return upper_bound;
}

float FuzzyGenerator::getLowerBound(){
    return lower_bound;
}