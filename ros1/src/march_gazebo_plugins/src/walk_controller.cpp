// Copyright 2019 Project March.

#include <march_gazebo_plugins/walk_controller.h>
#include <gazebo/physics/physics.hh>

namespace gazebo
{
WalkController::WalkController(physics::ModelPtr model) : ObstacleController(model)
{
  this->swing_step_size_ = 0.7;  // This estimate will be adjusted every step

  this->p_pitch_ = 220;
  this->d_pitch_ = 70000;
  this->p_roll_ = 250;
  this->d_roll_ = 70000;
  this->p_yaw_ = 1500;
  this->d_yaw_ = 70000;
}

}  // namespace gazebo
