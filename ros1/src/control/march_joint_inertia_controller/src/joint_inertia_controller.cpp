// Copyright 2020 Project March.
#include "march_joint_inertia_controller/joint_inertia_controller.h"
#include "march_joint_inertia_controller/inertia_estimator.h"
#include <angles/angles.h>
#include <cmath>
#include <pluginlib/class_list_macros.hpp>
#include <trajectory_msgs/JointTrajectory.h>

namespace joint_inertia_controller {
InertiaController::~InertiaController()
{
    sub_command_.shutdown();
}

bool InertiaController::init(
    hardware_interface::EffortJointInterface* hw, ros::NodeHandle& nh)
{
    if (!nh.getParam("joint_name", joint_name_)) {
        ROS_ERROR("No joint_names specified");
        return false;
    }

    sub_command_ = nh.subscribe<std_msgs::Float64>(
        "command", 1, &InertiaController::commandCB, this);

    // Get joint handle from hardware interface
    joint_ = hw->getHandle(joint_name_);

    ros::Duration first(0.004);
    inertia_estimator_.fillBuffers(
        joint_.getVelocity(), joint_.getEffort(), first);

    // Get URDF info about joint
    urdf::Model urdf;
    if (!urdf.initParamWithNodeHandle("robot_description", nh)) {
        ROS_ERROR("Failed to parse urdf file");
        return false;
    }
    joint_urdf_ = urdf.getJoint(joint_name_);
    if (!joint_urdf_) {
        ROS_ERROR("Could not find joint '%s' in urdf", joint_name_.c_str());
        return false;
    }

    return true;
}

void InertiaController::setGains(const double& p, const double& i,
    const double& d, const double& i_max, const double& i_min,
    const bool& antiwindup)
{
    pid_controller_.setGains(p, i, d, i_max, i_min, antiwindup);
}

void InertiaController::getGains(double& p, double& i, double& d, double& i_max,
    double& i_min, bool& antiwindup)
{
    pid_controller_.getGains(p, i, d, i_max, i_min, antiwindup);
}

void InertiaController::getGains(
    double& p, double& i, double& d, double& i_max, double& i_min)
{
    bool dummy;
    pid_controller_.getGains(p, i, d, i_max, i_min, dummy);
}

void InertiaController::printDebug()
{
    pid_controller_.printValues();
}

std::string InertiaController::getJointName()
{
    return joint_.getName();
}

double InertiaController::getPosition()
{
    return joint_.getPosition();
}

void InertiaController::starting(const ros::Time& /* time */)
{
    double pos_command = joint_.getPosition();

    command_struct_.position = pos_command;
    command_struct_.has_velocity = false;

    command_.initRT(command_struct_);

    pid_controller_.reset();
}

void InertiaController::update(
    const ros::Time& /* time */, const ros::Duration& period)
{
    command_struct_ = *(command_.readFromRT());
    double command_position = command_struct_.position;
    double command_velocity = command_struct_.velocity;
    bool has_velocity_ = command_struct_.has_velocity;

    double error, vel_error;
    double commanded_effort;

    double current_position = joint_.getPosition();

    if (joint_urdf_->type == urdf::Joint::REVOLUTE) {
        angles::shortest_angular_distance_with_large_limits(current_position,
            command_position, joint_urdf_->limits->lower,
            joint_urdf_->limits->upper, error);
    } else if (joint_urdf_->type == urdf::Joint::CONTINUOUS) {
        error = angles::shortest_angular_distance(
            current_position, command_position);
    } else // prismatic
    {
        error = command_position - current_position;
    }

    // Decide which of the two PID computeCommand() methods to call
    if (has_velocity_) {
        // Compute velocity error if a non-zero velocity command was given
        vel_error = command_velocity - joint_.getVelocity();

        // Set the PID error and compute the PID command with nonuniform
        // time step size. This also allows the user to pass in a precomputed
        // derivative error.
        commanded_effort
            = pid_controller_.computeCommand(error, vel_error, period);
    } else {
        // Set the PID error and compute the PID command with nonuniform
        // time step size.
        commanded_effort = pid_controller_.computeCommand(error, period);
    }

    inertia_estimator_.fillBuffers(
        joint_.getVelocity(), joint_.getEffort(), period);
    inertia_estimator_.inertiaEstimate();
    // TO DO: Provide lookup table for gain selection
    // TO DO: apply PID control

    joint_.setCommand(commanded_effort);
}
void InertiaController::stopping(const ros::Time& /* time */)
{
}

void InertiaController::commandCB(const std_msgs::Float64ConstPtr& msg)
{
    setCommand(msg->data);
}

void InertiaController::setCommand(double pos_command, double vel_command)
{
    command_struct_.position = pos_command;
    command_struct_.velocity = vel_command;
    command_struct_.has_velocity = true;

    command_.writeFromNonRT(command_struct_);
}

void InertiaController::setCommand(double pos_command)
{
    command_struct_.position = pos_command;
    command_struct_.has_velocity
        = false; // Flag to ignore the velocity command since our setCommand
                 // method did not include it

    // the writeFromNonRT can be used in RT, if you have the guarantee that
    //  * no non-rt thread is calling the same function (we're not subscribing
    //  to ros callbacks)
    //  * there is only one single rt thread
    command_.writeFromNonRT(command_struct_);
}

} // namespace joint_inertia_controller
PLUGINLIB_EXPORT_CLASS(joint_inertia_controller::InertiaController,
    controller_interface::ControllerBase);
