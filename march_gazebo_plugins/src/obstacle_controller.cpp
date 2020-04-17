// Copyright 2019 Project March.

#include <march_gazebo_plugins/obstacle_controller.h>
#include <ros/ros.h>

namespace gazebo
{
ObstacleController::ObstacleController(physics::ModelPtr model)
  : model_(model)
  , subgait_name_("home_stand")
  , subgait_changed_(true)
  , error_x_last_timestep_(0)
  , error_y_last_timestep_(0)
  , error_yaw_last_timestep_(0)
{
  this->foot_left_ = this->model_->GetLink("ankle_plate_left");
  this->foot_right_ = this->model_->GetLink("ankle_plate_right");

  this->subgait_name_ = "home_stand";
  this->subgait_changed_ = true;

  this->mass = 0.0;
  for (auto const& link : this->model_->GetLinks())
  {
    this->mass += link->GetInertial()->Mass();
  }
}

void ObstacleController::newSubgait(const march_shared_resources::GaitActionGoalConstPtr& _msg)
{
  if (this->subgait_name_ == "right_open" or this->subgait_name_ == "right_swing" or
      this->subgait_name_ == "left_swing")
  {
    // Exponential smoothing with alpha = 0.2
    this->swing_step_size_ = 0.8 * this->swing_step_size_ + 0.4 * std::abs(this->foot_right_->WorldPose().Pos().X() -
                                                                           this->foot_left_->WorldPose().Pos().X());
  }

  if (this->subgait_name_ == "home_stand" and _msg->goal.current_subgait.name.substr(0, 4) == "left")
  {
    ROS_WARN("Gait starts with left. CoM controller plugin might not work properly.");
  }
  this->subgait_name_ = _msg->goal.current_subgait.name;
  this->subgait_duration_ =
      _msg->goal.current_subgait.duration.sec + 0.000000001 * _msg->goal.current_subgait.duration.nsec;
  this->subgait_start_time_ = this->model_->GetWorld()->SimTime().Double();
  this->subgait_changed_ = true;
}

// Called by the world update start event
ignition::math::v4::Vector3<double> ObstacleController::GetCom()
{
  ignition::math::v4::Vector3<double> com(0.0, 0.0, 0.0);
  for (auto const& link : this->model_->GetLinks())
  {
    com += link->WorldCoGPose().Pos() * link->GetInertial()->Mass();
  }
  return com / this->mass;
}

void ObstacleController::update(ignition::math::v4::Vector3<double>& torque_all,
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

  double goal_position_x;
  double goal_position_y;

  this->getGoalPosition(time_since_start, goal_position_x, goal_position_y);

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
