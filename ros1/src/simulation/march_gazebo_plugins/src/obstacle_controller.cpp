// Copyright 2019 Project March.

#include <march_gazebo_plugins/obstacle_controller.h>
#include <ros/ros.h>
namespace gazebo
{
ObstacleController::ObstacleController(physics::ModelPtr model)
  : model_(std::move(model))
  , HOME_STAND("home_stand")
  , STAND_IDLE("idle_stand")
  , SIT_IDLE("idle_sit")
  , subgait_start_time_(0)
  , subgait_duration_(0)
  , subgait_changed_(true)
  , balance_(false)
  , p_yaw_(0)
  , d_yaw_(0)
  , p_yaw_off_(0)
  , d_yaw_off_(0)
  , p_pitch_(0)
  , d_pitch_(0)
  , p_pitch_off_(0)
  , d_pitch_off_(0)
  , p_roll_(0)
  , d_roll_(0)
  , p_roll_off_(0)
  , d_roll_off_(0)
  , error_x_last_timestep_(0)
  , error_y_last_timestep_(0)
  , error_yaw_last_timestep_(0)
  , goal_position_x(0)
  , goal_position_y(0)

{
  foot_left_ = model_->GetLink("ankle_plate_left");
  foot_right_ = model_->GetLink("ankle_plate_right");
  ros::param::get("balance", balance_);
  // As long as no sitting gait is executed, the default to use when no subgait is idle_stand
  subgait_name_ = HOME_STAND;
  default_subgait_name_ = STAND_IDLE;
  subgait_changed_ = true;

  mass = 0.0;
  for (auto const& link : model_->GetLinks())
  {
    mass += link->GetInertial()->Mass();
  }
}

void ObstacleController::newSubgait(const march_shared_msgs::CurrentGaitConstPtr& msg)
{
  if (subgait_name_ == "right_open" or subgait_name_ == "right_swing" or
      subgait_name_ == "left_swing")
  {
    // Exponential smoothing with alpha = 0.2
    swing_step_size_ = 0.8 * swing_step_size_ + 0.4 * std::abs(foot_right_->WorldPose().Pos().X() -
                                                                           foot_left_->WorldPose().Pos().X());
  }

  if (subgait_name_ == HOME_STAND and msg->subgait.substr(0, 4) == "left")
  {
    ROS_WARN("Gait starts with left. CoM controller plugin might not work properly.");
  }
  subgait_name_ = msg->subgait.empty() ? default_subgait_name_ : msg->subgait;
  subgait_duration_ = msg->duration.toSec();
  subgait_start_time_ = model_->GetWorld()->SimTime().Double();
  subgait_changed_ = true;
}

// Called by the world update start event
ignition::math::v6::Vector3<double> ObstacleController::GetCom()
{
  ignition::math::v6::Vector3<double> com(0.0, 0.0, 0.0);
  for (auto const& link : model_->GetLinks())
  {
    com += link->WorldCoGPose().Pos() * link->GetInertial()->Mass();
  }
  return com / mass;
}

void ObstacleController::update(ignition::math::v6::Vector3<double>& torque_left,
                                ignition::math::v6::Vector3<double>& torque_right)
{
  // Note: the exo moves in the negative x direction, and the right leg is in
  // the positive y direction

  double time_since_start = model_->GetWorld()->SimTime().Double() - subgait_start_time_;
  if (time_since_start > 1.05 * subgait_duration_)
  {
    if (subgait_name_ != default_subgait_name_ and subgait_name_ != HOME_STAND)
    {
        subgait_changed_ = true;
    }
    subgait_name_ = default_subgait_name_;
  }

  auto model_com = GetCom();

  getGoalPosition(time_since_start);
  double error_x = model_com.X() - goal_position_x;
  double error_y = model_com.Y() - goal_position_y;
  double error_yaw = foot_left_->WorldPose().Rot().Z();


  // Deactivate d if the subgait just changed to avoid effort peaks when the target function jumps
  if (subgait_changed_)
  {
    error_x_last_timestep_ = error_x;
    error_y_last_timestep_ = error_y;
    error_yaw_last_timestep_ = error_yaw;
    subgait_changed_ = false;
  }
  double p_pitch_actual, p_roll_actual, p_yaw_actual, d_pitch_actual, d_roll_actual, d_yaw_actual;

  // roll, pitch and yaw are defined in
  // https://docs.projectmarch.nl/doc/march_packages/march_simulation.html#torque-application
  // turn (bodge) off plug-in at right time when balance is set to true
  if (balance_ == true && subgait_name_ != HOME_STAND && subgait_name_ != STAND_IDLE)
  {
    p_pitch_actual = p_pitch_off_;
    p_roll_actual = p_roll_off_;
    p_yaw_actual = p_yaw_off_;
    d_pitch_actual = d_pitch_off_;
    d_roll_actual = d_roll_off_;
    d_yaw_actual = d_yaw_off_;
  }
  else
  {
    p_pitch_actual = p_pitch_;
    p_roll_actual = p_roll_;
    p_yaw_actual = p_yaw_;
    d_pitch_actual = d_pitch_;
    d_roll_actual = d_roll_;
    d_yaw_actual = d_yaw_;
  }


  double T_pitch = -p_pitch_actual * error_x - d_pitch_actual * (error_x - error_x_last_timestep_);
  double T_roll = p_roll_actual * error_y + d_roll_actual * (error_y - error_y_last_timestep_);
  double T_yaw = -p_yaw_actual * error_yaw - d_yaw_actual * (error_yaw - error_yaw_last_timestep_);

  if (subgait_name_.substr(0, 4) == "left")
  {
    torque_right = ignition::math::v6::Vector3<double>(T_roll, T_pitch, T_yaw);
    torque_left = ignition::math::v6::Vector3<double>(0, 0, 0);
  }
  else
  {
    torque_left = ignition::math::v6::Vector3<double>(T_roll, T_pitch, T_yaw);
    torque_right = ignition::math::v6::Vector3<double>(0, 0, 0);
  }

  error_x_last_timestep_ = error_x;
  error_y_last_timestep_ = error_y;
  error_yaw_last_timestep_ = error_yaw;

}

void ObstacleController::getGoalPosition(double time_since_start)
{
  // Left foot is stable unless subgait name starts with left
  auto stable_foot_pose = foot_left_->WorldCoGPose().Pos();
  auto swing_foot_pose = foot_right_->WorldCoGPose().Pos();
  if (subgait_name_.substr(0, 4) == "left")
  {
    stable_foot_pose = foot_right_->WorldCoGPose().Pos();
    swing_foot_pose = foot_left_->WorldCoGPose().Pos();
  }
  // Goal position is determined from the location of the stable foot
  goal_position_y = 0.75 * stable_foot_pose.Y() + 0.25 * swing_foot_pose.Y();

  // If the exoskeleton is busy sitting down, move the CoM behind the stable foot
  // Set 'sitting' as the new default state
  if (subgait_name_ == 'sit_down') {
      default_subgait_name_ = SIT_IDLE;
      goal_position_x = stable_foot_pose.X() + 0.1 * swing_step_size_;
  }
  // If the exoskeleton has sat down or is changing while sitting, keep the CoM behind the stable foot
  else if (subgait_name_ == SIT_IDLE || subgait_name_ == 'sit_home' || subgait_name_ == 'prepare_stand_up') {
      goal_position_x = stable_foot_pose.X() + 0.2 * swing_step_size_;
  }
  // If the exoskeleton is busy standing up, move the CoM forward again (relative when sitting down)
  // Set 'Standing' as the new defualt state
  else if (subgait_name_ == 'stand_up') {
      default_subgait_name_ = STAND_IDLE;
      goal_position_x = stable_foot_pose.X() + 0.1 * swing_step_size_;
  }
  // If the exoskeleton has stood up, keep the CoM on the stable foot
  else if (subgait_name_ == STAND_IDLE) {
    goal_position_x = stable_foot_pose.X();
  }

  if (subgait_name_.substr(0, 6) != "freeze")
  {
    goal_position_x = stable_foot_pose.X();
    // Start goal position a quarter step size behind the stable foot
    // Move the goal position forward with v = 0.5 * swing_step_size/subgait_duration
    if (subgait_name_.substr(subgait_name_.size() - 4) == "open")
    {
      goal_position_x += -0.25 * time_since_start * swing_step_size_ / subgait_duration_;
    }
    else if (subgait_name_.substr(subgait_name_.size() - 5) == "swing")
    {
      goal_position_x += 0.25 * swing_step_size_ - 0.5 * time_since_start * swing_step_size_ / subgait_duration_;
    }
    else if (subgait_name_.substr(subgait_name_.size() - 5) == "close")
    {
      goal_position_x += 0.25 * swing_step_size_ - 0.25 * time_since_start * swing_step_size_ / subgait_duration_;
    }
  }
}

}  // namespace gazebo
