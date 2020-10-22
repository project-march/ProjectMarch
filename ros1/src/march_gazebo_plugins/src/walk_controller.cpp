// Copyright 2019 Project March.

#include <march_gazebo_plugins/walk_controller.h>
#include <gazebo/physics/physics.hh>

namespace gazebo
{
WalkController::WalkController(physics::ModelPtr model) : ObstacleController(model)
{
  swing_step_size_ = 0.7;  // This estimate will be adjusted every step

  // The non-off_ values are tune so that the exo stays upright, the _off_ are set so the exo falls over. These _off_
  // values can be changed if one wishes the plugin to deliver some torque even when turned off.
  p_pitch_ = 120;
  p_pitch_off_ = 0;
  d_pitch_ = 40000;
  d_pitch_off_ = 0;

  p_roll_ = 150;
  p_roll_off_ = 0;
  d_roll_ = 40000;
  d_roll_off_ = 0;

  p_yaw_ = 1000;
  p_yaw_off_ = 0;
  d_yaw_ = 40000;
  d_yaw_off_ = 0;
}

}  // namespace gazebo
