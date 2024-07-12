#ifndef GAIN_SCHEDULER_HPP
#define GAIN_SCHEDULER_HPP
#pragma once

#include "../../march_mode_machine/include/march_mode_machine/exo_mode.hpp"
#include <sensor_msgs/msg/joint_state.hpp>
#include <yaml-cpp/yaml.h>
#include "rclcpp/rclcpp.hpp"
#include <unordered_map>
#include <iostream>
#include <map>

using joints_with_gains = std::vector<std::tuple<std::string, double, double, double>>;

class GainScheduler {
    public:
        GainScheduler();
        ~GainScheduler() = default;
        explicit GainScheduler(std::string system_type); 
        joints_with_gains getJointAngleGains(const sensor_msgs::msg::JointState::SharedPtr& joint_states);
        joints_with_gains getConstantGains(std::string gains_type);
        joints_with_gains setStanceSwingLegGains(joints_with_gains joints_with_gains, uint8_t current_stance_leg);
        void setGaitConfiguration(const ExoMode new_gait_type);
        std::string m_scheduling_variable;
        bool m_use_stance_swing_leg_gains;

        static constexpr std::size_t joint_name_index = 0;
        static constexpr std::size_t p_gain_index = 1;
        static constexpr std::size_t i_gain_index = 2;
        static constexpr std::size_t d_gain_index = 3;
    private: 
        std::tuple<std::string, double, double, double> getSpecificJointAngleGains(const std::string& joint_name, double joint_angle);
        ExoMode m_gait_type; 
        std::string m_config_base;
        YAML::Node m_joints_config;
        joints_with_gains m_joints_with_gains;
};
#endif // GAIN_SCHEDULER_HPP