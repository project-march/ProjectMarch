// Copyright 2019 Project March.

#include <march_gazebo_plugins/walk_controller.h>
#include <gazebo/physics/physics.hh>

namespace gazebo
{
WalkController::WalkController(physics::ModelPtr model)
  : ObstacleController(model), error_x_last_timestep_(0), error_y_last_timestep_(0), error_yaw_last_timestep_(0)
{
  this->swing_step_size_ = 0.7;  // This estimate will be adjusted every step

  this->p_pitch_ = 60;
  this->d_pitch_ = 10;
  this->p_roll_ = 150;
  this->d_roll_ = 0;
  this->p_yaw_ = 500;
  this->d_yaw_ = 25;
}

void WalkController::update(ignition::math::v4::Vector3<double>& torque_all,
                            ignition::math::v4::Vector3<double>& torque_stable)
{
  // Note: the exo moves in the negative x direction, and the right leg is in
  // the positive y direction

  double time_since_start = this->model_->GetWorld()->SimTime().Double() - this->subgait_start_time_;
  if (time_since_start > this->subgait_duration_)
  {
    this->subgait_name_ = "home_stand";
    this->subgait_changed_ = true;
  }

  auto model_com = this->GetCom();

  // Left foot is stable unless subgait name starts with left
  auto stable_foot_pose = this->foot_left_->WorldCoGPose().Pos();
  std::string stable_side = "left";
  if (this->subgait_name_.substr(0, 4) == "left")
  {
    stable_foot_pose = this->foot_right_->WorldCoGPose().Pos();
    stable_side = "right";
  }

  // Goal position is determined from the location of the stable foot
  double goal_position_x = stable_foot_pose.X();
  double goal_position_y = stable_foot_pose.Y();

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

  double error_x = model_com.X() - goal_position_x;
  double error_y = model_com.Y() - goal_position_y;
  double error_yaw = this->foot_left_->WorldPose().Rot().Z();

  // Deactivate d if the subgait just changed to peaks when the target function jumps
  if (this->subgait_changed_)
  {
    this->error_y_last_timestep_ = error_x;
    this->error_y_last_timestep_ = error_y;
    this->subgait_changed_ = false;
  }

  double T_pitch = -this->p_pitch_ * error_x - this->d_pitch_ * (error_x - this->error_x_last_timestep_);
  double T_roll = this->p_roll_ * error_y + this->d_roll_ * (error_y - this->error_y_last_timestep_);
  double T_yaw = -this->p_yaw_ * error_yaw - this->d_yaw_ * (error_yaw - this->error_yaw_last_timestep_);

  torque_all = ignition::math::v4::Vector3<double>(0, T_pitch, T_yaw);  // -roll, pitch, -yaw
  torque_stable = ignition::math::v4::Vector3<double>(T_roll, 0, 0);    // -roll, pitch, -yaw

  this->error_x_last_timestep_ = error_x;
  this->error_y_last_timestep_ = error_y;
  this->error_yaw_last_timestep_ = error_yaw;
}
}  // namespace gazebo
