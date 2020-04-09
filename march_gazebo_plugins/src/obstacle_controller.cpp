// Copyright 2019 Project March.

#include <obstacle_controller.h>

namespace gazebo
{
ObstacleController::ObstacleController(physics::ModelPtr model) : model_(model)
{
  this->foot_left = this->model_->GetLink("ankle_plate_left");
  this->foot_right = this->model_->GetLink("ankle_plate_right");

  this->subgait_name = "home_stand";
}

void ObstacleController::newSubgait(const march_shared_resources::GaitActionGoalConstPtr& _msg)
{
  if (this->subgait_name == "right_open" or this->subgait_name == "right_swing" or this->subgait_name == "left_swing")
  {
    this->swing_step_size = 0.8 * this->swing_step_size + 0.4 * std::abs(this->foot_right->WorldPose().Pos().X() -
                                                                         this->foot_left->WorldPose().Pos().X());
  }

  this->subgait_name = _msg->goal.current_subgait.name;
  this->subgait_duration =
      _msg->goal.current_subgait.duration.sec + 0.000000001 * _msg->goal.current_subgait.duration.nsec;
  this->subgait_start_time = this->model_->GetWorld()->SimTime().Double();
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
