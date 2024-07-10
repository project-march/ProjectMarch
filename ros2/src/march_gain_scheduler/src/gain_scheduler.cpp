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

    m_config = YAML::LoadFile(m_config_base + "/exemplary_config.yaml");
}

joints_with_gains GainScheduler::getConstantGains(std::string gains_type) {
    joints_with_gains joints;
    YAML::Node joints_config = m_config["joints"];
    
    for (YAML::const_iterator it = joints_config.begin(); it != joints_config.end(); ++it) {
        std::string joint_name = it->first.as<std::string>();
        std::vector<double> gains = joints_config[joint_name][gains_type].as<std::vector<double>>();

        joints.push_back(std::make_tuple(joint_name, gains[0], gains[1], gains[2]));
    }
    return joints;
}


// Method to get the PID values for all joints
joints_with_gains GainScheduler::getJointAngleGains(const sensor_msgs::msg::JointState::SharedPtr& joint_states) {
    joints_with_gains joints;
    
    for (size_t i = 0; i < joint_states->name.size(); ++i) {
        const auto& joint_name = joint_states->name[i];
        double joint_angle = joint_states->position[i];
        joints.push_back(getSpecificJointAngleGains(joint_name, joint_angle));
    }
    return joints;
}

std::tuple<std::string, double, double, double> GainScheduler::getSpecificJointAngleGains(const std::string& joint_name, double joint_angle) {
    const auto& joint_angle_ranges_config = m_config["joints"][joint_name]["joint_angle_ranges"];
    const auto number_of_ranges = joint_angle_ranges_config.size();
    std::vector<double> gains;

    for (size_t i = 0; i < number_of_ranges; ++i) {
        double min_range = joint_angle_ranges_config[i][0][0].as<double>();
        double max_range = joint_angle_ranges_config[i][0][1].as<double>();
        if (joint_angle >= min_range && joint_angle < max_range) {
            gains = joint_angle_ranges_config[i][1].as<std::vector<double>>();
        }
    }
    return std::make_tuple(joint_name, gains[0], gains[1], gains[2]);
}

// Method to set the config path
void GainScheduler::setConfigPath(const ExoMode &new_gait_type) {
    m_gait_type = new_gait_type;

}
