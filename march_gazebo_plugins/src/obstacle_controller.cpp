// Copyright 2019 Project March.

#include <march_gazebo_plugins/obstacle_controller.h>

namespace gazebo
{
ObstacleController::ObstacleController(physics::ModelPtr model) : model_(model)
{
  this->foot_left_ = this->model_->GetLink("ankle_plate_left");
  this->foot_right_ = this->model_->GetLink("ankle_plate_right");

  this->subgait_name_ = "home_stand";
  this->subgait_changed_ = true;
}

void ObstacleController::newSubgait(const march_shared_resources::GaitActionGoalConstPtr& _msg)
{
  if (this->subgait_name_ == "right_open" or this->subgait_name_ == "right_swing" or
      this->subgait_name_ == "left_swing")
  {
    this->swing_step_size_ = 0.8 * this->swing_step_size_ + 0.4 * std::abs(this->foot_right_->WorldPose().Pos().X() -
                                                                           this->foot_left_->WorldPose().Pos().X());
  }

  this->subgait_name_ = _msg->goal.current_subgait.name;
  this->subgait_duration_ =
      _msg->goal.current_subgait.duration.sec + 0.000000001 * _msg->goal.current_subgait.duration.nsec;
  this->subgait_start_time_ = this->model_->GetWorld()->SimTime().Double();
  this->subgait_changed_ = true;
}

// Called by the world update start event
double ObstacleController::getMass()
{
  double mass = 0.0;
  for (auto const& link : this->model_->GetLinks())
  {
    mass += link->GetInertial()->Mass();
  }
  return mass;
}

// Called by the world update start event
ignition::math::v4::Vector3<double> ObstacleController::GetCom()
{
  ignition::math::v4::Vector3<double> com(0.0, 0.0, 0.0);
  for (auto const& link : this->model_->GetLinks())
  {
    com += link->WorldCoGPose().Pos() * link->GetInertial()->Mass();
  }
  return com / this->getMass();
}
}  // namespace gazebo
