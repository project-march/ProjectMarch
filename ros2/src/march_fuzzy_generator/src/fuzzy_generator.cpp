#include "fuzzy_generator/fuzzy_generator.hpp"

FuzzyGenerator::FuzzyGenerator(){}

FuzzyGenerator::FuzzyGenerator(std::string config_path)
{
    m_config = YAML::LoadFile(config_path);
    m_torque_ranges = getTorqueRanges();
}


// Method to get the constant weights
std::vector<std::tuple<std::string, float, float>> FuzzyGenerator::getConstantWeights() 
{
    std::vector<std::tuple<std::string, float, float>> constant_joint_weights;
  
    for (const auto& joint_torque_ranges : m_torque_ranges) {
        const std::string joint_name = std::get<m_joint_name_index>(joint_torque_ranges);
        const float torque_weight = std::get<m_torque_weight_index>(joint_torque_ranges);
        const float position_weight = 1 - torque_weight;

        constant_joint_weights.push_back(std::make_tuple(joint_name, position_weight, torque_weight));
    }
    return constant_joint_weights;
}


// Method to calculate the foot height weights
std::vector<std::tuple<std::string, float, float>> FuzzyGenerator::calculateFootHeightWeights(const march_shared_msgs::msg::FeetHeightStamped::SharedPtr& both_foot_heights)
{    
    m_lower_bound = m_config["height_bounds"]["lower_bound"].as<double>();
    m_upper_bound = m_config["height_bounds"]["upper_bound"].as<double>();

    if (both_foot_heights == nullptr) {
        throw std::runtime_error("No foot height received.");
    }  
    double left_foot_height = both_foot_heights->heights[m_left_foot_index];
    double right_foot_height = both_foot_heights->heights[m_right_foot_index];

    if (left_foot_height < 0 || right_foot_height < 0) {
        throw std::runtime_error("Negative foot heights received for both feet.");
    }

    // Set the lowest foot to 0
    if (left_foot_height < right_foot_height) {
        left_foot_height = 0;
    } else {
        right_foot_height = 0;
    }
    return calculateVariableWeights(left_foot_height, right_foot_height);
}


// Method to calculate the stance swing leg weights
std::vector<std::tuple<std::string, float, float>> FuzzyGenerator::getAnkleTorques(double left_ankle_torque, double right_ankle_torque)
{
    m_lower_bound = m_config["torque_bounds"]["lower_bound"].as<double>();
    m_upper_bound = m_config["torque_bounds"]["upper_bound"].as<double>();

    if (left_ankle_torque == 0 && right_ankle_torque == 0) {
        throw std::runtime_error("No ankle torques received.");
    }
    return calculateVariableWeights(left_ankle_torque, right_ankle_torque);
}


// Method to calculate the variable weights
std::vector<std::tuple<std::string, float, float>> FuzzyGenerator::calculateVariableWeights(double left_leg_parameter, double right_leg_parameter) 
{
    std::vector<std::tuple<std::string, float, float>> joint_weights_vector;
    joint_weights_vector.reserve(m_torque_ranges.size()); 
    double fuzzy_parameter;

    // for each joint in the leg, calculate the torque weight and position weight
    for (const auto& joint_torque_ranges : m_torque_ranges) {    
        const std::string joint_name = std::get<m_joint_name_index>(joint_torque_ranges);
        const float minimum_torque_percentage = std::get<m_min_torque_index>(joint_torque_ranges);
        const float maximum_torque_percentage = std::get<m_max_torque_index>(joint_torque_ranges);
        const float torque_range = maximum_torque_percentage - minimum_torque_percentage;

        if (joint_name.find("left") != std::string::npos) {
            fuzzy_parameter = left_leg_parameter;
        } else if (joint_name.find("right") != std::string::npos) {
            fuzzy_parameter = right_leg_parameter;
        } else {
            throw std::runtime_error("Joint not found");
        }

        // calculate how far the parameter is in the 'fuzzy shifting range'
        const float fuzzy_percentage = (m_upper_bound - fuzzy_parameter) / (m_upper_bound - m_lower_bound);
        float torque_weight = torque_range * fuzzy_percentage;
        torque_weight = std::max(minimum_torque_percentage, std::min(torque_weight, maximum_torque_percentage));
        const float position_weight = 1 - torque_weight;

        joint_weights_vector.emplace_back(joint_name, position_weight, torque_weight);  
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


// Method to get the joint names
std::vector<std::tuple<std::string, float, float>> FuzzyGenerator::returnPositionWeights()
{
    std::vector<std::tuple<std::string, float, float>> position_weights_vector;
    position_weights_vector.reserve(m_torque_ranges.size()); 

    for (const auto& joint_torque_ranges : m_torque_ranges) {
        position_weights_vector.push_back(std::make_tuple(std::get<m_joint_name_index>(joint_torque_ranges), 1, 0));
    }
    return position_weights_vector;
}


// Method to set the config path
// TODO: update the config (paths) for the rest of the gaits
void FuzzyGenerator::setConfigPath(const ExoMode &new_gait_type) 
{
    m_gait_type = new_gait_type;

    if (new_gait_type == static_cast<ExoMode>(m_walk_index)) {
        m_config = YAML::LoadFile("~/src/march_fuzzy_generator/config/walk_weights_tsu.yaml");  
        m_control_type = "position";
    } else if (new_gait_type == static_cast<ExoMode>(m_sideways_walk_index)) {
        m_config = YAML::LoadFile("~/src/march_fuzzy_generator/config/sideways_walk_weights_tsu.yaml");
        m_control_type = "stance_swing_leg";    
    } else {
        m_config = YAML::LoadFile("~/src/march_fuzzy_generator/config/default_weights_tsu.yaml");
        m_control_type = "constant";
    }

    m_torque_ranges = getTorqueRanges();
}
