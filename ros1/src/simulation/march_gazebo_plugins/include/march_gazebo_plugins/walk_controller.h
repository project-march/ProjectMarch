// Copyright 2019 Project March.
#ifndef MARCH_GAZEBO_PLUGINS_WALK_CONTROLLER_H
#define MARCH_GAZEBO_PLUGINS_WALK_CONTROLLER_H

#include <march_gazebo_plugins/obstacle_controller.h>
#include "yaml-cpp/yaml.h"


namespace gazebo
{
class WalkController : public ObstacleController
{
public:
  explicit WalkController(physics::ModelPtr model);
//  std::vector<std::string> com_levels;
//  YAML::Node com_levels_tree;
};
}  // namespace gazebo

#endif  // MARCH_GAZEBO_PLUGINS_WALK_CONTROLLER_H
