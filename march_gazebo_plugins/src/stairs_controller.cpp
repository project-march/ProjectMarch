// Copyright 2019 Project March.

#include <march_gazebo_plugins/stairs_controller.h>
#include <gazebo/physics/physics.hh>

namespace gazebo
{
StairsController::StairsController(physics::ModelPtr model) : ObstacleController(model)
{
  this->swing_step_size_ = 0.7;  // This estimate will be adjusted every step

  this->p_pitch_ = 60;
  this->d_pitch_ = 10;
  this->p_roll_ = 150;
  this->d_roll_ = 0;
  this->p_yaw_ = 500;
  this->d_yaw_ = 25;
}

void StairsController::getGoalPosition(double time_since_start, double& goal_position_x, double& goal_position_y)
{
  // Left foot is stable unless subgait name starts with left
  auto stable_foot_pose = this->foot_left_->WorldCoGPose().Pos();
  if (this->subgait_name_.substr(0, 4) == "left")
  {
    stable_foot_pose = this->foot_right_->WorldCoGPose().Pos();
  }

  // Goal position is determined from the location of the stable foot
  goal_position_x = stable_foot_pose.X();
  goal_position_y = stable_foot_pose.Y();

  // Start goal position a quarter step size behind the stable foot
  // Move the goal position forward with v = 0.5 * swing_step_size/subgait_duration
  if (this->subgait_name_.substr(this->subgait_name_.size() - 4) == "open")
  {
    goal_position_x += -0.25 * time_since_start * this->swing_step_size_ / this->subgait_duration_;
  }
  else if (this->subgait_name_.substr(this->subgait_name_.size() - 5) == "swing")
  {
    goal_position_x +=
        0.25 * this->swing_step_size_ - 0.5 * time_since_start * this->swing_step_size_ / this->subgait_duration_;
  }
  else if (this->subgait_name_.substr(this->subgait_name_.size() - 5) == "close")
  {
    goal_position_x +=
        0.25 * this->swing_step_size_ - 0.25 * time_since_start * this->swing_step_size_ / this->subgait_duration_;
  }
}
}  // namespace gazebo
