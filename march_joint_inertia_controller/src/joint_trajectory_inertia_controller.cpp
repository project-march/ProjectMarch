// Copyright 2020 Project March.
#include "march_joint_inertia_controller/joint_inertia_controller.h"
#include "march_joint_inertia_controller/inertia_estimator.h"
#include <pluginlib/class_list_macros.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <math.h>
#include <angles/angles.h>
#include <pluginlib/class_list_macros.hpp>

using joint_limits_interface::JointLimits;
using joint_limits_interface::PositionJointSoftLimitsHandle;
using joint_limits_interface::PositionJointSoftLimitsInterface;
using joint_limits_interface::SoftJointLimits;

HardwareInterfaceAdapter::HardwareInterfaceAdapter() : joint_handles_ptr(0)
{
}

bool HardwareInterfaceAdapter::init(hardware_interface::PositionJointInterface* hw, ros::NodeHandle& nh)
{
  if (!nh.getParam("joint_name", joint_name))
  {
    ROS_ERROR("No joint_names specified");
    return false;
  }

  // May be totally unnecessary as we have direct access to deired state in updateCommand method...
  sub_command_ = nh.subscribe<std_msgs::Float64>("command", 1, &InertiaController::commandCB, this);

  // Get joint handle from hardware interface
  joint_ = hw->getHandle(joint_name);

  // Get URDF info about joint
  urdf::Model urdf;
  if (!urdf.initParamWithNodeHandle("robot_description", nh))
  {
    ROS_ERROR("Failed to parse urdf file");
    return false;
  }
  joint_urdf_ = urdf.getJoint(joint_name);
  if (!joint_urdf_)
  {
    ROS_ERROR("Could not find joint '%s' in urdf", joint_name.c_str());
    return false;
  }

  return true;
}

void HardwareInterfaceAdapter::setGains(const double& p, const double& i, const double& d, const double& i_max,
                                        const double& i_min, const bool& antiwindup)
{
  pid_controller_.setGains(p, i, d, i_max, i_min, antiwindup);
}

void HardwareInterfaceAdapter::getGains(double& p, double& i, double& d, double& i_max, double& i_min, bool& antiwindup)
{
  pid_controller_.getGains(p, i, d, i_max, i_min, antiwindup);
}

void HardwareInterfaceAdapter::getGains(double& p, double& i, double& d, double& i_max, double& i_min)
{
  bool dummy;
  pid_controller_.getGains(p, i, d, i_max, i_min, dummy);
}

void HardwareInterfaceAdapter::printDebug()
{
  pid_controller_.printValues();
}

std::string HardwareInterfaceAdapter::getJointName()
{
  return joint_.getName();
}

double HardwareInterfaceAdapter::getPosition()
{
  return joint_.getPosition();
}

// Wait what is difference between starting and init????
void HardwareInterfaceAdapter::starting(const ros::Time& /* time */)
{
  double pos_command = joint_.getPosition();

  command_struct_.position_ = pos_command;

  command_.initRT(command_struct_);

  pid_controller_.reset();

  // init_pos_ = this->march_robot_->getJoint(j).getPosition();
  for (size_t i = 1; i < velocity_array_.size(); ++i)
  {
    velocity_array_[i] = 0.0;
  }

  for (size_t i = 1; i < acceleration_array_.size(); ++i)
  {
    acceleration_array_[i] = 0.0;
  }

  for (size_t i = 1; i < joint_torque_.size(); ++i)
  {
    joint_torque_[i] = 0.0;
  }
}

void HardwareInterfaceAdapter::updateCommand(const ros::Time& /* time */, const ros::Duration& period /* period */,
                                             const joint_trajectory_controller::State& desired_state,
                                             const joint_trajectory_controller::State& state_error)
{
  double command_position = desired_state.position_;
  double command_velocity = desired_state.velocity_;

  double error, vel_error;
  double commanded_effort;

  double current_position = joint_.getPosition();

  // Example says to chweck limits here, but we already do that in the validate() function in the HWI_node

  if (joint_urdf_->type == urdf::Joint::REVOLUTE)
  {
    angles::shortest_angular_distance_with_large_limits(current_position, command_position, joint_urdf_->limits->lower,
                                                        joint_urdf_->limits->upper, error);
  }
  else if (joint_urdf_->type == urdf::Joint::CONTINUOUS)
  {
    error = angles::shortest_angular_distance(current_position, command_position);
  }
  else  // prismatic
  {
    error = command_position - current_position;
  }

  // Set the PID error and compute the PID command with nonuniform
  // time step size.
  commanded_effort = pid_controller_.computeCommand(error, period);

  //  this->fill_buffers(period);
  //  this->inertia_estimate();
  // TO DO: Provide lookup table for gain selection
  // TO DO: apply PID control

  joint_.setCommand(commanded_effort);

  // Example says to update state p[ublisher but we already do that somewhere else?

  loop_count_++;
}
void HardwareInterfaceAdapter::stopping(const ros::Time& /* time */)
{
}

void HardwareInterfaceAdapter::commandCB(const std_msgs::Float64ConstPtr& msg)
{
  setCommand(msg->data);
}

void HardwareInterfaceAdapter::setCommand(double pos_command)
{
  command_struct_.position_ = pos_command;
  command_struct_.has_velocity_ =
      false;  // Flag to ignore the velocity command since our setCommand method did not include it

  // the writeFromNonRT can be used in RT, if you have the guarantee that
  //  * no non-rt thread is calling the same function (we're not subscribing to ros callbacks)
  //  * there is only one single rt thread
  command_.writeFromNonRT(command_struct_);
}

namespace inertia_trajectory_controller
{
typedef joint_trajectory_controller::JointTrajectoryController<trajectory_interface::QuinticSplineSegment<double>,
                                                               hardware_interface::EffortJointInterface>
    JointTrajectoryController;

}  // namespace inertia_trajectory_controller

PLUGINLIB_EXPORT_CLASS(inertia_trajectory_controller::JointTrajectoryController, controller_interface::ControllerBase);
