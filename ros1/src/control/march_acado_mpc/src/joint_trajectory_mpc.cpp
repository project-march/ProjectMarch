#include "joint_trajectory_mpc.hpp"
#include "model_predictive_controller.hpp"
#include <pluginlib/class_list_macros.hpp>

#include <iostream>


bool ModelPredictiveControllerInterface::init(std::vector<hardware_interface::JointHandle>& joint_handles, ros::NodeHandle& nh)
{
  joint_handles_ptr_ = &joint_handles;
  num_joints_ = joint_handles_ptr_->size();

  //initialize the model predictive controllers
  model_predictive_controllers_.resize(num_joints_);
  for (unsigned int i = 0; i < num_joints_; ++i)
  {
    model_predictive_controllers_[i].init();
  }

  return true;
}

void ModelPredictiveControllerInterface::starting(const ros::Time& /*time*/)
{
  if (!joint_handles_ptr_) {return;}

  // zero commands
  for (unsigned int i = 0; i < num_joints_; ++i)
  {
    (*joint_handles_ptr_)[i].setCommand(0.0);
  }
}

void ModelPredictiveControllerInterface::updateCommand(const ros::Time& /*time*/, const ros::Duration& period,
                   const joint_trajectory_controller::State& /*desired state*/,
                   const joint_trajectory_controller::State& state_error)
{
  num_joints_ = joint_handles_ptr_->size();

  // Preconditions
  if (!joint_handles_ptr_)
  {
    return;
  }
  assert(num_joints_ == state_error.position.size());
  assert(num_joints_ == state_error.velocity.size());

  // Update effort command
  for (unsigned int i = 0; i < num_joints_; ++i)
  {
    // Set current joint state
    state = {(*joint_handles_ptr_)[i].getPosition(), (*joint_handles_ptr_)[i].getVelocity()};
    model_predictive_controllers_[i].x0 = state;

    // Calculate mpc control signal
    model_predictive_controllers_[i].controller();

    // Set command variable
//    command = state_error.position[i]*1000;

    command = model_predictive_controllers_[i].u;

    // Apply command variable
    (*joint_handles_ptr_)[i].setCommand(command);
  }

}

void ModelPredictiveControllerInterface::stopping(const ros::Time& /*time*/)
{

}


// Exporting the controller plugin
namespace model_predictive_trajectory_controller
{
typedef joint_trajectory_controller::JointTrajectoryController<trajectory_interface::QuinticSplineSegment<double>,
hardware_interface::EffortJointInterface>

    JointTrajectoryController;

}  // model_predictive_trajectory_controller

PLUGINLIB_EXPORT_CLASS(model_predictive_trajectory_controller::JointTrajectoryController, controller_interface::ControllerBase);
