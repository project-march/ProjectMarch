// Copyright 2020 Project March.
#include "march_joint_inertia_controller/joint_inertia_controller.h"
#include <math.h>

namespace joint_inertia_controller
{
bool InertiaController::init(hardware_interface::PositionJointInterface* hw, ros::NodeHandle& nh)
{
  std::string joint_name;
  if (!nh.getParam("joint_name", joint_name))
  {
    ROS_ERROR("No joint_name specified");
    return false;
  }
  joint_ = hw->getHandle(joint_name);
  return true;
}

void InertiaController::starting(const ros::Time& /* time */)
{
  init_pos_ = joint_.getPosition();
}

void InertiaController::update(const ros::Time& /* time */, const ros::Duration& /* period */)
{
  double dpos = init_pos_ + 10 * sin(ros::Time::now().toSec());
  double cpos = joint_.getPosition();
  joint_.setCommand(-10 * (cpos - dpos));
}
void InertiaController::stopping(const ros::Time& /* time */)
{
}
}  // namespace joint_inertia_controller
PLUGINLIB_EXPORT_CLASS(joint_inertia_controller::InertiaController, controller_interface::ControllerBase);
