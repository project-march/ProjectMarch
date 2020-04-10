// Copyright 2019 Project March.

#include <stairs_controller.h>
#include <gazebo/physics/physics.hh>

namespace gazebo
{
StairsController::StairsController(physics::ModelPtr model) : ObstacleController(model)
{
  this->swing_step_size = 0.7;  // This estimate will be adjusted every step

  this->p_pitch = 60;
  this->d_pitch = 10;
  this->p_roll = 150;
  this->d_roll = 0;
  this->p_yaw = 500;
  this->d_yaw = 25;

  this->error_x_last_timestep = 0;
  this->error_y_last_timestep = 0;
  this->error_yaw_last_timestep = 0;
}

void StairsController::update(ignition::math::v4::Vector3<double>& torque_all,
                              ignition::math::v4::Vector3<double>& torque_stable)
{
  // Note: the exo moves in the negative x direction, and the right leg is in
  // the positive y direction

  double time_since_start = this->model_->GetWorld()->SimTime().Double() - subgait_start_time;
  auto model_com = this->GetCom();

  // Left foot is stable unless subgait name starts with left
  auto stable_foot_pose = this->foot_left->WorldCoGPose().Pos();
  std::string stable_side = "left";
  if (this->subgait_name.substr(0, 4) == "left")
  {
    stable_foot_pose = this->foot_right->WorldCoGPose().Pos();
    stable_side = "right";
  }

  // Goal position is determined from the location of the stable foot
  double goal_position_x = stable_foot_pose.X();
  double goal_position_y = stable_foot_pose.Y();

  // Start goal position a quarter step size behind the stable foot
  // Move the goal position forward with v = 0.5 * swing_step_size/subgait_duration
  if (this->subgait_name.substr(this->subgait_name.size() - 4) == "open")
  {
    goal_position_x += -0.25 * time_since_start * swing_step_size / subgait_duration;
  }
  else if (this->subgait_name.substr(this->subgait_name.size() - 5) == "swing")
  {
    goal_position_x += 0.25 * swing_step_size - 0.5 * time_since_start * swing_step_size / subgait_duration;
  }
  else if (this->subgait_name.substr(this->subgait_name.size() - 5) == "close")
  {
    goal_position_x += 0.25 * swing_step_size - 0.25 * time_since_start * swing_step_size / subgait_duration;
  }

  if (time_since_start > subgait_duration)
  {
    this->subgait_name = "home_stand";
  }

  double error_x = model_com.X() - goal_position_x;
  double error_y = model_com.Y() - goal_position_y;
  double error_yaw = this->foot_left->WorldPose().Rot().Z();

  double T_pitch = -this->p_pitch * error_x - this->d_pitch * (error_x - this->error_x_last_timestep);
  double T_roll = this->p_roll * error_y + this->d_roll * (error_y - this->error_y_last_timestep);
  double T_yaw = -this->p_yaw * error_yaw - this->d_yaw * (error_yaw - this->error_yaw_last_timestep);

  torque_all = ignition::math::v4::Vector3<double>(0, T_pitch, T_yaw);  // -roll, pitch, -yaw
  torque_stable = ignition::math::v4::Vector3<double>(T_roll, 0, 0);    // -roll, pitch, -yaw

  this->error_x_last_timestep = error_x;
  this->error_y_last_timestep = error_y;
  this->error_yaw_last_timestep = error_yaw;
}
}  // namespace gazebo
