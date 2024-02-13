//
// Created by rixt on 12-4-23.
//

#ifndef MARCH_FUZZY_GENERATOR_HPP
#define MARCH_FUZZY_GENERATOR_HPP
#pragma once

#include "march_shared_msgs/msg/foot_heights.hpp"
#include "../../march_mode_machine/include/march_mode_machine/exo_mode.hpp"
#include "rclcpp/rclcpp.hpp"
#include <yaml-cpp/yaml.h>


class FuzzyGenerator {
    public:
        FuzzyGenerator();
        FuzzyGenerator(std::string config_path);
        void setConfigPath(const exoMode &new_gait_type);
        std::vector<std::tuple<std::string, float, float>> getConstantWeights();
        std::vector<std::tuple<std::string, float, float>> calculateFootHeightWeights(const march_shared_msgs::msg::FootHeights::SharedPtr& both_foot_heights);
        std::vector<std::tuple<std::string, float, float>> calculateStanceSwingLegWeights(std::vector<double> stance_swing);
        std::vector<std::string> m_joint_names;

    private:
        double m_lower_bound;
        double m_upper_bound;
        YAML::Node m_config;
        exoMode m_gait_type;
        std::vector<std::tuple<std::string, float, float, float>> m_torque_ranges;
        void setJointParameters();
        std::vector<std::tuple<std::string, float, float, float>> getTorqueRanges();
        void getJointNames();

};

#endif // MARCH_FUZZY_GENERATOR_HPP