#include "joint_trajectory_template.hpp"
#include <pluginlib/class_list_macros.hpp> // what is this

#include <iostream> // for printing things (c_out)
#include <numeric> // for something
#include <string>
#include <vector>

// WARNING! UNSAFE! // Why is this warning here? ask Thijs Raymakers / internet
// If you initialize this object, YOU, the caller, have to ensure
// that both `joint_handles` and `nh` have a lifetime that is at
// least as long as the lifetime of this object. Otherwise,
// undefined behavior WILL happen.
bool TemplateControllerInterface::init(
    std::vector<hardware_interface::JointHandle>& joint_handles,
    ros::NodeHandle& nh)
{
    // Get joint handles and the amount of joints to control
    joint_handles_ptr_ = &joint_handles;
    num_joints_ = joint_handles.size();

    // Get the names of the joints to control
    // This is not very much needed, but nice for mixed control etc
    std::vector<std::string> joint_names;
    nh.getParam("joints", joint_names);

    return true;
}

// Function that dictates what to do when the controller is started by the
// controller manager
// Nice starting sounds will need to be here :)
void TemplateControllerInterface::starting(const ros::Time& /*time*/)
{
    if (!joint_handles_ptr_) {
        return;
    }

    // zero commands
    for (unsigned int i = 0; i < num_joints_; ++i) {
        (*joint_handles_ptr_)[i].setCommand(/*command=*/0.0);
    }
}

// Function that calculates the command that needs to be send to each joint
void TemplateControllerInterface::updateCommand(const ros::Time& /*time*/,
    const ros::Duration& period,
    const std::vector<joint_trajectory_controller::State>& /*desired_states*/,
    const joint_trajectory_controller::State& state_error)
{
    // Preconditions
    if (!joint_handles_ptr_) {
        return;
    }
    assert(num_joints_ == state_error.position.size());
    assert(num_joints_ == state_error.velocity.size());

    // Set command
    for (int i = 0; i < num_joints_; ++i) {
        // Apply command
        command = 0; // Will become calculation of some sort
        (*joint_handles_ptr_)[i].setCommand(command);
    }
}

// Function that dictates what to do when the controller is stopped by the
// controller manager
void TemplateControllerInterface::stopping(const ros::Time& /*time*/)
{
    // zero commands
    for (int i = 0; i < num_joints_; ++i) {
        (*joint_handles_ptr_)[i].setCommand(/*command=*/0.0);
    }
}

// Exporting the controller plugin
namespace template_trajectory_controller {
typedef joint_trajectory_controller::JointTrajectoryController<
    trajectory_interface::QuinticSplineSegment<double>,
    hardware_interface::EffortJointInterface>

    JointTrajectoryController;

} // namespace template_trajectory_controller

PLUGINLIB_EXPORT_CLASS(
    template_trajectory_controller::JointTrajectoryController,
    controller_interface::ControllerBase);
