//
// Created by rixt on 12-4-23.
//

#include "fuzzy_generator/fuzzy_generator.hpp"

FuzzyGenerator::FuzzyGenerator(){
}

FuzzyGenerator::FuzzyGenerator(std::string config_path){
    m_config = YAML::LoadFile(config_path);
    setJointParameters();
    getJointNames();
}

std::vector<std::tuple<std::string, float, float>> FuzzyGenerator::calculateFootHeightWeights(const march_shared_msgs::msg::FootHeights::SharedPtr& both_foot_heights){
    const unsigned int left_foot_index = 0;
    const unsigned int right_foot_index = 1;
    const unsigned int joint_name_index = 0;
    const unsigned int min_torque_index = 1;
    const unsigned int max_torque_index = 2;

    std::vector<std::tuple<std::string, float, float>> joint_weights_vector;
    double left_foot_height;
    double right_foot_height;

    if (both_foot_heights == nullptr) {
        throw std::runtime_error("No foot height received.");
    } else {
        left_foot_height = both_foot_heights->heights[left_foot_index];
        right_foot_height = both_foot_heights->heights[right_foot_index];
    }   

    if (left_foot_height < 0 || right_foot_height < 0) {
        throw std::runtime_error("Negative foot height received.");
    }

    // Set the lowest foot to 0
    if (left_foot_height < right_foot_height) {
        left_foot_height = 0;
    } else {
        right_foot_height = 0;
    }

    // for each joint in the leg, calculate the torque weight and position weight
    for (const auto& joint_torque_ranges : m_torque_ranges) {
        
        const std::string joint_name = std::get<joint_name_index>(joint_torque_ranges);
        const float minimum_torque_percentage = std::get<min_torque_index>(joint_torque_ranges);
        const float maximum_torque_percentage = std::get<max_torque_index>(joint_torque_ranges);
        const float torque_range = maximum_torque_percentage - minimum_torque_percentage;

        // getting the correct foot height and leg
        float foot_height;
    
        if (joint_name.find("left") != std::string::npos) {
            foot_height = left_foot_height;
        } else if (joint_name.find("right") != std::string::npos) {
            foot_height = right_foot_height;
        } else {
            throw std::runtime_error("Joint not found");
        }

        // calculate how far the foot is in the 'fuzzy shifting range'
        const float height_percentage = (m_upper_bound - foot_height) / (m_upper_bound - m_lower_bound);
        float torque_weight = torque_range * height_percentage;
        torque_weight = std::max(minimum_torque_percentage, std::min(torque_weight, maximum_torque_percentage));
        const float position_weight = 1 - torque_weight;

        joint_weights_vector.push_back(std::make_tuple(joint_name, position_weight, torque_weight));
    }

    return joint_weights_vector;
}

//  Method to get the torque ranges
std::vector<std::tuple<std::string, float, float, float>> FuzzyGenerator::getTorqueRanges()
{
    std::vector<std::tuple<std::string, float, float, float>> joint_torque_ranges;
    YAML::Node joints_config = m_config["joints"];

    for (YAML::const_iterator it = joints_config.begin(); it != joints_config.end(); ++it) {
        
        std::string joint_name = it->first.as<std::string>();
        const float min_torque = joints_config[joint_name]["minimum_torque"].as<float>();
        const float max_torque = joints_config[joint_name]["maximum_torque"].as<float>();
        const float const_torque = joints_config[joint_name]["constant_weight"].as<float>();

        joint_torque_ranges.push_back(std::make_tuple(joint_name, min_torque, max_torque, const_torque));
    }
    
    return joint_torque_ranges;
}

// Method to set the config path
// TODO: update the config (paths) for the rest of the gaits
void FuzzyGenerator::setConfigPath(const exoMode &new_gait_type) {
    m_gait_type = new_gait_type;
    std::cout << "Gait type set to: " << m_gait_type << '\n';

    if (new_gait_type == static_cast<exoMode>(0)) {
        m_config = YAML::LoadFile("src/march_fuzzy_generator/config/sit_weights_tsu.yaml");  
    } else if (new_gait_type == static_cast<exoMode>(1)) {
        m_config = YAML::LoadFile("src/march_fuzzy_generator/config/stand_weights_tsu.yaml");
    } else if (new_gait_type == static_cast<exoMode>(2)) {
        m_config = YAML::LoadFile("src/march_fuzzy_generator/config/walk_weights_tsu.yaml");
    } else {
        throw std::runtime_error("Gait type not found");
    }

    setJointParameters();
}

std::vector<std::tuple<std::string, float, float>> FuzzyGenerator::getConstantWeights() {

    std::vector<std::tuple<std::string, float, float>> constant_joint_weights;

    for (const auto& joint_torque_ranges : m_torque_ranges) {
        
        const std::string joint_name = std::get<0>(joint_torque_ranges);
        const float torque_weight = std::get<3>(joint_torque_ranges);
        const float position_weight = 1 - torque_weight;

        constant_joint_weights.push_back(std::make_tuple(joint_name, position_weight, torque_weight));
    }

    return constant_joint_weights;
}

// Method to set the joint parameters
void FuzzyGenerator::setJointParameters() {
    m_torque_ranges = getTorqueRanges();
    m_lower_bound = m_config["bounds"]["lower_bound"].as<double>();
    m_upper_bound = m_config["bounds"]["upper_bound"].as<double>();
}

void FuzzyGenerator::getJointNames() {

    std::vector<std::string> joint_names; 

    for (const auto& joint_torque_ranges : m_torque_ranges) {
        
        const std::string joint_name = std::get<0>(joint_torque_ranges);
        joint_names.push_back(joint_name);
    }

    m_joint_names = joint_names; 
}


std::vector<std::tuple<std::string, float, float>> FuzzyGenerator::calculateStanceSwingLegWeights(std::vector<double> stance_swing){

    // stance-swing leg scheduling logic 

    return std::vector<std::tuple<std::string, float, float>>();
}
