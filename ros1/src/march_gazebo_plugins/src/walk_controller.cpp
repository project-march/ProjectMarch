// Copyright 2019 Project March.

#include <march_gazebo_plugins/walk_controller.h>
#include <gazebo/physics/physics.hh>

namespace gazebo
{
WalkController::WalkController(physics::ModelPtr model) : ObstacleController(model)
{
  swing_step_size_ = 0.7;  // This estimate will be adjusted every step

  p_pitch_ = 220;
  d_pitch_ = 70000;
  p_roll_ = 250;
  d_roll_ = 70000;
  p_yaw_ = 1500;
  d_yaw_ = 70000;
}

}  // namespace gazebo
