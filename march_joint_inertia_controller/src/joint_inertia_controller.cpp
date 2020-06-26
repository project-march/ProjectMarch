// Copyright 2020 Project March.
#include "march_joint_inertia_controller/joint_inertia_controller.h"
#include <math.h>

using joint_limits_interface::JointLimits;
using joint_limits_interface::PositionJointSoftLimitsHandle;
using joint_limits_interface::PositionJointSoftLimitsInterface;
using joint_limits_interface::SoftJointLimits;

namespace joint_inertia_controller
{
bool InertiaController::init(hardware_interface::PositionJointInterface* hw, ros::NodeHandle& nh)
{
  if (!nh.getParam("joint_names", joint_names_))
  {
    ROS_ERROR("No joint_names specified");
    return false;
  }
  num_joints_ = joint_names_.size();
  joint_ = hw->getHandle(joint_names_);

  // setup acceleration and torque arrays

  // Setup Butterworth filter

  return true;
}

void InertiaController::starting(const ros::Time& /* time */)
{
  init_pos_ = joint_.getPosition();
}

void InertiaController::update(const ros::Time& /* time */, const ros::Duration& /* period */)
{
  joint_.setCommand(0);
}
void InertiaController::stopping(const ros::Time& /* time */)
{
}

// Estimate the inertia using the acceleration and torque
void InertiaController::inertia_estimate()
{
}

// Calculate a discrete derivative of the speed measurements
void InertiaController::discrete_speed_derivative(const ros::Duration&)
{
}

}  // namespace joint_inertia_controller
PLUGINLIB_EXPORT_CLASS(joint_inertia_controller::InertiaController, controller_interface::ControllerBase);
