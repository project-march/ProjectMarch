// Copyright 2020 Project March.
#include <angles/angles.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include "march_joint_inertia_controller/joint_trajectory_inertia_controller.h"
#include <math.h>
#include <pluginlib/class_list_macros.h>
#include <pluginlib/class_list_macros.hpp>
#include <trajectory_msgs/JointTrajectory.h>

namespace inertia_trajectory_controller
{
typedef joint_trajectory_controller::JointTrajectoryController<trajectory_interface::QuinticSplineSegment<double>,
                                                               hardware_interface::EffortJointInterface>
    JointTrajectoryController;

}  // namespace inertia_trajectory_controller

PLUGINLIB_EXPORT_CLASS(inertia_trajectory_controller::JointTrajectoryController, controller_interface::ControllerBase);
