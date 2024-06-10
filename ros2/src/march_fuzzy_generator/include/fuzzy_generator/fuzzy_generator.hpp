#ifndef MARCH_FUZZY_GENERATOR_HPP
#define MARCH_FUZZY_GENERATOR_HPP
#pragma once

#include "../../march_mode_machine/include/march_mode_machine/exo_mode.hpp"
#include "march_shared_msgs/msg/feet_height_stamped.hpp"
#include "march_shared_msgs/msg/joint_motor_controller_state.hpp"
#include "rclcpp/rclcpp.hpp"
#include <yaml-cpp/yaml.h>


class FuzzyGenerator {
    public:
        FuzzyGenerator();
        FuzzyGenerator(std::string system_type);
        void setConfigPath(const ExoMode &new_gait_type);
        std::vector<std::tuple<std::string, float, float>> getConstantWeights();
        std::vector<std::tuple<std::string, float, float>> calculateFootHeightWeights(const march_shared_msgs::msg::FeetHeightStamped::SharedPtr& both_foot_heights);
        std::vector<std::tuple<std::string, float, float>> getAnkleTorques(double left_ankle_torque, double right_ankle_torque); 
        std::vector<std::tuple<std::string, float, float>> returnPositionWeights();      
        std::string m_control_type = "position"; // default value

    private:
        std::string m_system_type;
        std::string m_package_path;
        YAML::Node m_config;
        double m_lower_bound;
        double m_upper_bound;
        ExoMode m_gait_type;
        std::vector<std::tuple<std::string, float, float, float>> m_torque_ranges;
        std::vector<std::tuple<std::string, float, float, float>> getTorqueRanges();
        std::vector<std::tuple<std::string, float, float>> calculateVariableWeights(double left_leg_parameter, double right_leg_parameter);
        
        static constexpr std::size_t m_left_foot_index = 0;
        static constexpr std::size_t m_right_foot_index = 1;
        static constexpr std::size_t m_joint_name_index = 0;
        static constexpr std::size_t m_min_torque_index = 1;
        static constexpr std::size_t m_max_torque_index = 2;
        static constexpr std::size_t m_torque_weight_index = 3;
        static constexpr std::size_t m_walk_index = 2;
        static constexpr std::size_t m_sideways_walk_index = 5; 
};

#endif // MARCH_FUZZY_GENERATOR_HPP