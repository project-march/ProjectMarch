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
// #include <yaml-cpp/yaml.h>

class FuzzyGenerator {
public:
    FuzzyGenerator();

    std::vector<std::tuple<std::string, float, float>>  calculateWeights(std::string leg, float foot_height);
    std::vector<std::tuple<std::string, float, float>>  getTorqueRanges(std::string leg);

    float getUpperBound();
    float getLowerBound();



private:
    double upper_bound; //TODO: add actual value
    double lower_bound; //TODO: add actual value

    // YAML::Node config_;    
};

#endif //MARCH_FUZZY_GENERATOR_HPP