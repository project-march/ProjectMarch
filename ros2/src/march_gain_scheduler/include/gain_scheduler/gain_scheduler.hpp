#pragma once

#include "../../march_mode_machine/include/march_mode_machine/exo_mode.hpp"
#include <sensor_msgs/msg/joint_state.hpp>
#include <yaml-cpp/yaml.h>
#include "rclcpp/rclcpp.hpp"
#include <iostream>
#include <map>

class GainScheduler {

    public:
        GainScheduler();
        ~GainScheduler() = default;
        explicit GainScheduler(std::string config_path); 
        std::vector<std::tuple<std::string, double, double, double>> getAllPidValues();
        std::vector<std::tuple<std::string, double, double, double>> getAllJointStatePidValues(const sensor_msgs::msg::JointState::SharedPtr& joint_states);
        std::vector<std::tuple<std::string, double, double, double>> getInterpolatedPidValues();
        void setConfigPath(const exoMode &new_gait_type);
        void startInterpolation() {m_interpolating = true;}
        void stopInterpolation() {m_interpolating = false;}      
        bool isInterpolating() {return m_interpolating;}
        void incrementTimeStep() {m_time_step += 0.1;};
        int m_publish_time = 100; // in milliseconds
        double m_time_step = 0.0;
        double m_total_time = 0.89;
    private: 
        std::tuple<std::string, double, double, double> getPidValues(const std::string& joint, double current_position);
        exoMode m_gait_type; 
        YAML::Node m_config;
        std::vector<std::tuple<std::string, double, double, double>> m_old_pid_values;
        std::vector<std::tuple<std::string, double, double, double>> m_new_pid_values;
        bool m_interpolating = false;   
        sensor_msgs::msg::JointState::SharedPtr m_last_joint_state;  
        double m_default_position = 0.0;
}; 