//
// Created by rixt on 12-4-23.
//

#ifndef MARCH_FUZZY_GENERATOR_HPP
#define MARCH_FUZZY_GENERATOR_HPP
#include "geometry_msgs/msg/point_stamped.hpp"
#include "march_shared_msgs/msg/feet_height_stamped.hpp"
#include "march_shared_msgs/msg/torque_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/int32.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <yaml-cpp/yaml.h>
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include <math.h>

class FuzzyGenerator {
public:
    FuzzyGenerator();
    FuzzyGenerator(rclcpp::Node::SharedPtr node);

    std::vector<std::tuple<std::string, float, float>> calculateWeights(std::vector<double> both_foot_heights);
    std::vector<std::tuple<std::string, float, float>> getTorqueRanges();

private:
    double lower_bound;
    double upper_bound;

    std::shared_ptr<rclcpp::Node> node_;

    YAML::Node config_;
};

#endif // MARCH_FUZZY_GENERATOR_HPP