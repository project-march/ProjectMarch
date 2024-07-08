#include "gain_scheduler/gain_scheduler.hpp"

GainScheduler::GainScheduler()
{
}

GainScheduler::GainScheduler(std::string config_path)
    : m_gait_type(static_cast<ExoMode>(0)), m_config(YAML::LoadFile(config_path))
{
}

// Method to get the PID values for a specific joint
std::tuple<std::string, double, double, double> GainScheduler::getPidValues(const std::string& joint, double current_position) {
    
    YAML::Node jointConfig = m_config["joints"][joint];
    if (!jointConfig) {
        throw std::runtime_error("Joint not found in config file");
    }

    int design_point_index = 0;

    for (std::size_t i = 0; i < jointConfig["design_points"].size(); i++) {
        if (jointConfig["design_points"][i]["design_point"].as<double>() <= current_position && 
            (i + 1 == jointConfig["design_points"].size() || current_position < jointConfig["design_points"][i + 1]["design_point"].as<double>())) {
            design_point_index = i;
            break;
        }
    }
    
    const double kp = jointConfig["design_points"][design_point_index]["pid_values"]["kp"].as<double>();
    const double ki = jointConfig["design_points"][design_point_index]["pid_values"]["ki"].as<double>();
    const double kd = jointConfig["design_points"][design_point_index]["pid_values"]["kd"].as<double>();    

    return std::make_tuple(joint, kp, ki, kd);
   
}

// Method to get the (interpolated) PID values for all joints
std::vector<std::tuple<std::string, double, double, double>> GainScheduler::getAllPidValues() {
        
    std::vector<std::tuple<std::string, double, double, double>> joints;
    YAML::Node joints_config = m_config["joints"];

        for (YAML::const_iterator it = joints_config.begin(); it != joints_config.end(); ++it) {
            std::string joint = it->first.as<std::string>();
            joints.push_back(getPidValues(joint,m_default_position));
        }
    
    return joints;
}

// Method to get the PID values for all joints
std::vector<std::tuple<std::string, double, double, double>> GainScheduler::getJointAngleGains(const sensor_msgs::msg::JointState::SharedPtr& joint_states) {
    std::vector<std::tuple<std::string, double, double, double>> joints;

    for (std::size_t i = 0; i < joint_states->name.size(); i++) {
        joints.push_back(getPidValues(joint_states->name[i], joint_states->position[i]));
    }

    m_last_joint_state = joint_states;
    return joints;
}

// Method to set the config path
void GainScheduler::setConfigPath(const ExoMode &new_gait_type) {
    m_gait_type = new_gait_type;

    if (new_gait_type == static_cast<ExoMode>(0)) {
        m_config = YAML::LoadFile("src/march_gain_scheduler/config/sit_gains.yaml");
    } else if (new_gait_type == static_cast<ExoMode>(1)) {
        m_config = YAML::LoadFile("src/march_gain_scheduler/config/stand_gains.yaml");
    } else {
        throw std::runtime_error("Gait type not found");
    }
}
