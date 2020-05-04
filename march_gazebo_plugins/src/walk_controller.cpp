// Copyright 2019 Project March.

#include <march_gazebo_plugins/walk_controller.h>
#include <gazebo/physics/physics.hh>

namespace gazebo
{
WalkController::WalkController(physics::ModelPtr model) : ObstacleController(model)
{
  this->swing_step_size_ = 0.7;  // This estimate will be adjusted every step
}

}  // namespace gazebo
