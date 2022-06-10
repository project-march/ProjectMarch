// Copyright 2019 Project March.

#include "yaml-cpp/yaml.h"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <march_gazebo_plugins/obstacle_controller.h>
#include <typeinfo>

namespace gazebo {
ObstacleController::ObstacleController(
    physics::ModelPtr model, std::map<std::string, int>& pd_values)
    : model_(std::move(model))
    , HOME_STAND(/*__s=*/"home_stand")
    , STAND_IDLE(/*__s=*/"idle_stand")
    , SIT_IDLE(/*__s=*/"idle_sit")
    , subgait_start_time_(0)
    , subgait_duration_(0)
    , subgait_changed_(true)
    , pd_values_(pd_values)
    , error_x_last_timestep_(0)
    , error_y_last_timestep_(0)
    , error_yaw_last_timestep_(0)
    , goal_position_x(0)
    , goal_position_y(0)

{
    foot_left_ = model_->GetLink("ankle_plate_left");
    foot_right_ = model_->GetLink("ankle_plate_right");

    // Get upper_leg_length from properties yaml:
    std::string path
        = ament_index_cpp::get_package_share_directory("march_description")
        + "/urdf/properties/properties_" + model_->GetName() + ".yaml";
    YAML::Node properties = YAML::LoadFile(path);
    double upper_leg_length_
        = properties["dimensions"]["upper_leg"]["length"].as<double>();
    halved_upper_leg_length_ = upper_leg_length_ / 2.0;

    // As long as no sitting gait is executed, the default to use when no
    // subgait is idle_stand
    subgait_name_ = HOME_STAND;
    default_subgait_name_ = STAND_IDLE;
    subgait_changed_ = true;

    mass = 0.0;
    for (auto const& link : model_->GetLinks()) {
        mass += link->GetInertial()->Mass();
    }
}

void ObstacleController::newSubgait(
    const march_shared_msgs::msg::CurrentGait::ConstSharedPtr& msg)
{
    if (subgait_name_ == "right_open" or subgait_name_ == "right_swing"
        or subgait_name_ == "left_swing") {
        // Exponential smoothing with alpha = 0.8
        double alpha = 0.8;
        swing_step_size_ = alpha * swing_step_size_
            + (1 - alpha) * 2
                * std::abs(foot_right_->WorldPose().Pos().X()
                    - foot_left_->WorldPose().Pos().X());
    }

    subgait_name_ = msg->subgait.empty() ? default_subgait_name_ : msg->subgait;
    subgait_duration_ = msg->duration.sec;
    subgait_start_time_ = model_->GetWorld()->SimTime().Double();
    subgait_changed_ = true;
}

// Called by the world update start event
ignition::math::v6::Vector3<double> ObstacleController::GetCom()
{
    ignition::math::v6::Vector3<double> com(0.0, 0.0, 0.0);
    for (auto const& link : model_->GetLinks()) {
        com += link->WorldCoGPose().Pos() * link->GetInertial()->Mass();
    }
    return com / mass;
}

void ObstacleController::update(
    ignition::math::v6::Vector3<double>& torque_left,
    ignition::math::v6::Vector3<double>& torque_right)
{
    // Note: the exo moves in the negative x direction, and the right leg is in
    // the positive y direction

    double time_since_start
        = model_->GetWorld()->SimTime().Double() - subgait_start_time_;
    if (time_since_start > 1.05 * subgait_duration_) {
        if (subgait_name_ != default_subgait_name_
            and subgait_name_ != HOME_STAND) {
            subgait_changed_ = true;
        }
        subgait_name_ = default_subgait_name_;
    }

    auto model_com = GetCom();

    getGoalPosition(time_since_start);
    double error_x = model_com.X() - goal_position_x;
    double error_y = model_com.Y() - goal_position_y;
    double error_yaw = foot_left_->WorldPose().Rot().Z();

    // Deactivate d if the subgait just changed to avoid effort peaks when the
    // target function jumps
    if (subgait_changed_) {
        error_x_last_timestep_ = error_x;
        error_y_last_timestep_ = error_y;
        error_yaw_last_timestep_ = error_yaw;
        subgait_changed_ = false;
    }

    double T_pitch
        = -pd_values_["p_pitch"] * error_x - pd_values_["d_pitch"] * (error_x - error_x_last_timestep_);
    double T_roll
        = pd_values_["p_roll"] * error_y + pd_values_["d_roll"] * (error_y - error_y_last_timestep_);
    double T_yaw
        = -pd_values_["p_yaw"] * error_yaw - pd_values_["d_yaw"] * (error_yaw - error_yaw_last_timestep_);

    if (subgait_name_.substr(/*__pos=*/0, /*__n=*/4) == "left") {
        torque_right
            = ignition::math::v6::Vector3<double>(T_roll, T_pitch, T_yaw);
        torque_left = ignition::math::v6::Vector3<double>(0, 0, 0);
    } else {
        torque_left
            = ignition::math::v6::Vector3<double>(T_roll, T_pitch, T_yaw);
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

    if (subgait_name_.find(/*__s=*/"left") != std::string::npos) {
        stable_foot_pose = foot_right_->WorldCoGPose().Pos();
        swing_foot_pose = foot_left_->WorldCoGPose().Pos();
    }

    // Y coordinate of the goal position is determined from the location of the
    // stable foot
    goal_position_y = 0.75 * stable_foot_pose.Y() + 0.25 * swing_foot_pose.Y();

    // X coordinate of the goal position is determined from the current subgait
    // the exoskeleton in executing if the exo skeleton is frozen, do not send a
    // new goal_position_x, keep it at previous value
    if (subgait_name_.find(/*__s=*/"freeze") == std::string::npos) {
        if (subgait_name_.find(/*__s=*/"sit") != std::string::npos
            || subgait_name_ == "prepare_stand_up") {
            getSitGoalPositionX(time_since_start, stable_foot_pose.X());
        } else if (subgait_name_.find(/*__s=*/"stand") != std::string::npos) {
            getStandGoalPositionX(time_since_start, stable_foot_pose.X());
        } else if (subgait_name_.find(/*__s=*/"right") != std::string::npos
            || subgait_name_.find(/*__s=*/"left") != std::string::npos) {
            getWalkGoalPositionX(time_since_start, stable_foot_pose.X());
        }
    }
}

void ObstacleController::getSitGoalPositionX(
    double time_since_start, double stable_foot_pose_x)
{
    goal_position_x = stable_foot_pose_x;
    // If the exoskeleton is busy sitting down, move the CoM behind the stable
    // foot Set 'sitting' as the new default state
    if (subgait_name_.substr(subgait_name_.size() - 8) == "sit_down") {
        default_subgait_name_ = SIT_IDLE;
        goal_position_x
            += halved_upper_leg_length_ * time_since_start / subgait_duration_;
    }
    // If the exoskeleton has sat down or is moving while sitting, keep the CoM
    // behind the stable foot
    else if (subgait_name_ == SIT_IDLE
        || subgait_name_.substr(subgait_name_.size() - 8) == "sit_home"
        || subgait_name_.substr(subgait_name_.size() - 16)
            == "prepare_stand_up") {
        goal_position_x
            += halved_upper_leg_length_; // and try using the hip position.
    }
}

void ObstacleController::getStandGoalPositionX(
    double time_since_start, double stable_foot_pose_x)
{
    goal_position_x = stable_foot_pose_x;
    // Set 'Standing' as the new default state
    default_subgait_name_ = STAND_IDLE;
    // If the exoskeleton is busy standing up, move the CoM forward again
    // (relative when sitting down)
    if (subgait_name_.substr(subgait_name_.size() - 8) == "stand_up") {
        goal_position_x += halved_upper_leg_length_
            * (1 - time_since_start / subgait_duration_);
    }
}

void ObstacleController::getWalkGoalPositionX(
    double time_since_start, double stable_foot_pose_x)
{
    goal_position_x = stable_foot_pose_x;
    // If the exoskeleton is executing an open gait (generally right_open),
    // move the CoM from the stable foot to a quarter step size in front.
    // quarter step sizes are used during the walk to avoid sending the CoM too
    // far from the stable foot
    if (subgait_name_.substr(subgait_name_.size() - 4) == "open") {
        goal_position_x
            += -0.25 * time_since_start * swing_step_size_ / subgait_duration_;
    }
    // During the swing phase, move the CoM from a quarter step size behind the
    // stable foot, to a quarter step size in front of the stable foot
    else if (subgait_name_.substr(subgait_name_.size() - 5) == "swing") {
        goal_position_x += 0.25 * swing_step_size_
            - 0.5 * time_since_start * swing_step_size_ / subgait_duration_;
    }
    // If the exosekelton in executing a close gait, move the CoM from a quarter
    // step size behind the stable foot, to the stable foot
    else if (subgait_name_.substr(subgait_name_.size() - 5) == "close") {
        goal_position_x += 0.25 * swing_step_size_
            - 0.25 * time_since_start * swing_step_size_ / subgait_duration_;
    }
}

} // namespace gazebo
