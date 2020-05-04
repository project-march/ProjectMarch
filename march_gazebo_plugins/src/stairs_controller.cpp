// Copyright 2019 Project March.

#include <march_gazebo_plugins/stairs_controller.h>
#include <gazebo/physics/physics.hh>

namespace gazebo
{
StairsController::StairsController(physics::ModelPtr model) : ObstacleController(model)
{
  this->swing_step_size_ = 0.56;  // This estimate will be adjusted every step
}
}  // namespace gazebo
