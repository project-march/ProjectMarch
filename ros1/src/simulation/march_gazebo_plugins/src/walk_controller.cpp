// Copyright 2019 Project March.

#include "yaml-cpp/yaml.h"
#include <gazebo/physics/physics.hh>
#include <march_gazebo_plugins/walk_controller.h>
#include <ros/package.h>
#include <utility>

namespace gazebo {
WalkController::WalkController(physics::ModelPtr model)
    : ObstacleController(std::move(model))
{
    swing_step_size_ = 0.7; // This estimate will be adjusted every step

    // Get the values from the configuration file
    std::string path = ros::package::getPath("march_gazebo_plugins")
        + "/config/com_levels.yaml";
    com_levels_tree = YAML::LoadFile(path);

    for (YAML::const_iterator it = com_levels_tree.begin();
         it != com_levels_tree.end(); ++it) {
        auto key = it->first.as<std::string>();
        com_levels.push_back(key);
    }

    // The non-balance_ values are tune so that the exo stays upright, the
    // _balance_ are set so for balance. These _balance_ values can be changed
    // if one wishes the plugin to deliver some torque even when turned balance.
    p_pitch_ = com_levels_tree["default"]["pitch"]["p"].as<double>();
    p_pitch_balance_ = com_levels_tree["off"]["pitch"]["p"].as<double>();
    d_pitch_ = com_levels_tree["default"]["pitch"]["d"].as<double>();
    d_pitch_balance_ = com_levels_tree["off"]["pitch"]["d"].as<double>();

    p_roll_ = com_levels_tree["default"]["roll"]["p"].as<double>();
    p_roll_balance_ = com_levels_tree["off"]["roll"]["p"].as<double>();
    d_roll_ = com_levels_tree["default"]["roll"]["d"].as<double>();
    d_roll_balance_ = com_levels_tree["off"]["roll"]["d"].as<double>();

    p_yaw_ = com_levels_tree["default"]["yaw"]["p"].as<double>();
    p_yaw_balance_ = com_levels_tree["off"]["yaw"]["p"].as<double>();
    d_yaw_ = com_levels_tree["default"]["yaw"]["d"].as<double>();
    d_yaw_balance_ = com_levels_tree["off"]["yaw"]["d"].as<double>();
}
} // namespace gazebo
