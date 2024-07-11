#include "gain_scheduler/gain_scheduler.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>

GainScheduler::GainScheduler()
{
}

GainScheduler::GainScheduler(std::string system_type)
{
    std::string package_path = ament_index_cpp::get_package_share_directory("march_gain_scheduler");

    // Set the base config path
    if (system_type == "tsu") {
        m_config_base = package_path + "/config/tsu";
    } else if (system_type == "exo") {
        m_config_base = package_path + "/config/exo";
    } else {
        RCLCPP_ERROR(rclcpp::get_logger("GainScheduler"), "System type %s is not defined.", system_type.c_str()); 
    }

    m_joints_config = YAML::LoadFile(m_config_base + "/exemplary_config.yaml")["joints"];
}

joints_with_gains GainScheduler::getConstantGains(std::string gains_type) {
    m_joints_with_gains.clear();
    
    for (YAML::const_iterator it = m_joints_config.begin(); it != m_joints_config.end(); ++it) {
        std::string joint_name = it->first.as<std::string>();
        std::vector<double> gains = m_joints_config[joint_name][gains_type].as<std::vector<double>>();

        m_joints_with_gains.push_back(std::make_tuple(joint_name, gains[p_gain_index], gains[i_gain_index], gains[d_gain_index]));
    }
    return m_joints_with_gains;
}

joints_with_gains GainScheduler::setStanceSwingLegGains(uint8_t current_stance_leg) {
    bool left_leg_stance = current_stance_leg == 1;
    bool right_leg_stance = current_stance_leg == 2;
    bool both_legs_stance = current_stance_leg == 3;

    for (size_t i = 0; i < m_joints_with_gains.size(); ++i) {
        std::string joint_name = std::get<joint_name_index>(m_joints_with_gains[i]);

        // Set the integral gain to 0.0 for the stance leg joints
        if ((left_leg_stance && joint_name.find("left") != std::string::npos) ||
            (right_leg_stance && joint_name.find("right") != std::string::npos) ||
            both_legs_stance) {
            std::get<i_gain_index>(m_joints_with_gains[i]) = 0.0;
        }
    }
    return m_joints_with_gains;
}

joints_with_gains GainScheduler::getJointAngleGains(const sensor_msgs::msg::JointState::SharedPtr& joint_states) {
    m_joints_with_gains.clear();
    
    for (size_t i = 0; i < joint_states->name.size(); ++i) {
        const auto& joint_name = joint_states->name[i];
        double joint_angle = joint_states->position[i];
        m_joints_with_gains.push_back(getSpecificJointAngleGains(joint_name, joint_angle));
    }
    return m_joints_with_gains;
}

std::tuple<std::string, double, double, double> GainScheduler::getSpecificJointAngleGains(const std::string& joint_name, double joint_angle) {
    const auto& joint_angle_ranges_config = m_joints_config[joint_name]["joint_angle_ranges"];
    const auto number_of_ranges = joint_angle_ranges_config.size();
    std::vector<double> gains;

    for (size_t i = 0; i < number_of_ranges; ++i) {
        const auto& range_config = joint_angle_ranges_config[i];
        const auto& range_values = range_config[0];
        const auto& gains_values = range_config[1];

        double min_range = range_values[0].as<double>();
        double max_range = range_values[1].as<double>();

        if (joint_angle >= min_range && joint_angle < max_range) {
            gains = gains_values.as<std::vector<double>>();
        }
    }
    return std::make_tuple(joint_name, gains[p_gain_index], gains[i_gain_index], gains[d_gain_index]);
}

// Method to set the config path
void GainScheduler::setConfigPath(const ExoMode &new_gait_type) {
    m_gait_type = new_gait_type;

}
