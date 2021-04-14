// Copyright 2019 Project March.
#ifndef MARCH_GAZEBO_PLUGINS_WALK_CONTROLLER_H
#define MARCH_GAZEBO_PLUGINS_WALK_CONTROLLER_H

#include <march_gazebo_plugins/obstacle_controller.h>
#include <ros/ros.h>

namespace gazebo {
class WalkController : public ObstacleController {
public:
    explicit WalkController(physics::ModelPtr model);
};
} // namespace gazebo

#endif // MARCH_GAZEBO_PLUGINS_WALK_CONTROLLER_H
