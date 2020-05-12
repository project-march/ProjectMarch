// Copyright 2020 Project March.
#include "../include/march_joint_inertia_controller/joint_inertia_controller.h"
#include <math.h>

namespace joint_inertia_controller_ns
{
// Controller initialization
bool InertiaController::init(hardware_interface::PositionJointInterface* hw, ros::NodeHandle& nh)
{
  // Retrieve the joint object to control
  std::string joint_name;
  if (!nh.getParam("joint_name", joint_name))
  {
    ROS_ERROR("No joint_name specified");
    return false;
  }
  joint_ = hw->getHandle(joint_name);
  return true;
}

// Controller startup
void InertiaController::starting(const ros::Time& /* time */)
{
  // Get initial position to use in the control procedure
  init_pos_ = joint_.getPosition();
}

// Controller running
void InertiaController::update(const ros::Time& /* time */, const ros::Duration& /* period */)
{
  //---Perform a sinusoidal motion for joint
  double dpos = init_pos_ + 10 * sin(ros::Time::now().toSec());
  double cpos = joint_.getPosition();
  joint_.setCommand(-10 * (cpos - dpos));  // Apply command to the selected joint
}
// Controller exiting
void InertiaController::stopping(const ros::Time& /* time */)
{
}
}  // namespace joint_inertia_controller_ns
PLUGINLIB_EXPORT_CLASS(joint_inertia_controller_ns::InertiaController, controller_interface::ControllerBase);
