#include "joint_trajectory_mpc.hpp"
#include <pluginlib/class_list_macros.hpp>

typedef HardwareInterfaceAdapter<hardware_interface::EffortJointInterface, joint_trajectory_controller::State> HWIA;

bool HWIA::init(std::vector<hardware_interface::JointHandle>& joint_handles, ros::NodeHandle& nh)
{
  joint_handles_ptr_ = &joint_handles;
  num_joints_ = joint_handles_ptr_->size();

  return true;
}

void HWIA::starting(const ros::Time& /*time*/)
{
  if (!joint_handles_ptr_) {return;}

  // zero commands
  for (unsigned int i = 0; i < num_joints_; ++i)
  {
    (*joint_handles_ptr_)[i].setCommand(0.0);
  }
}

void HWIA::updateCommand(const ros::Time& /*time*/, const ros::Duration& period,
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
    // simple P controller to test setCommand() and the trajectory controller
    const double command = state_error.position[i]*1000;
    (*joint_handles_ptr_)[i].setCommand(command);
  }

}

void HWIA::stopping(const ros::Time& /*time*/)
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
