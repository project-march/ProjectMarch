// Copyright 2019 Project March.

#include <march_gazebo_plugins/obstacle_controller.h>

#ifndef MARCH_GAZEBO_PLUGINS_WALK_CONTROLLER_H
#define MARCH_GAZEBO_PLUGINS_WALK_CONTROLLER_H

namespace gazebo
{
class WalkController : public ObstacleController
{
public:
  explicit WalkController(physics::ModelPtr model);
};
}  // namespace gazebo

#endif  // MARCH_RQT_GAIT_GENERATOR_OBSTACLECONTROLLER_H
