// Copyright 2019 Project March.

#include <march_gazebo_plugins/walk_controller.h>
#include <gazebo/physics/physics.hh>

namespace gazebo
{
WalkController::WalkController(physics::ModelPtr model) : ObstacleController(model)
{
  this->swing_step_size_ = 0.7;  // This estimate will be adjusted every step

  this->p_pitch_ = 120;
  this->d_pitch_ = 40000;
  this->p_roll_ = 85;
  this->d_roll_ = 40000;
  this->p_yaw_ = 1000;
  this->d_yaw_ = 40000;
}

}  // namespace gazebo
