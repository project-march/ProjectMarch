//
// Created by rixt on 12-4-23.
//

#ifndef MARCH_FUZZY_GENERATOR_HPP
#define MARCH_FUZZY_GENERATOR_HPP
#include "geometry_msgs/msg/point_stamped.hpp"
#include "march_shared_msgs/msg/feet_height_stamped.hpp"
#include "march_shared_msgs/msg/torque_stamped.hpp"
#include "std_msgs/msg/int32.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include <yaml-cpp/yaml.h>
#include <ament_index_cpp/get_package_share_directory.hpp>

inline static const char PATH_SEPARATOR =
#ifdef _WIN32
    '\\';
#else
    '/';
#endif

class FuzzyGenerator {
public:
    FuzzyGenerator();

    std::vector<std::tuple<std::string, float, float>>  calculateWeights(std::vector<double> both_foot_heights);
    std::vector<std::tuple<std::string, float, float>>  getTorqueRanges();
    std::string getStanceLeg(std::vector<double> both_foot_heights);
    bool isAscending(std::string current_leg);
    void updateVelocities();



private:
    double descending_upper_bound;
    double descending_lower_bound;
    double ascending_upper_bound;
    double ascending_lower_bound;

    std::vector<std::vector<float>> log{{},{}};

    float delta_avg_l = 0.0f;
    float delta_avg_r = 0.0f;
    float alpha = 0.2;

    YAML::Node config_;    
};

#endif //MARCH_FUZZY_GENERATOR_HPP