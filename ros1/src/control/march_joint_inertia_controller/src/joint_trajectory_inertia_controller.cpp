// Copyright 2020 Project March.
#include "march_joint_inertia_controller/joint_trajectory_inertia_controller.h"
#include <pluginlib/class_list_macros.hpp>

namespace inertia_trajectory_controller {
typedef joint_trajectory_controller::JointTrajectoryController<
    trajectory_interface::QuinticSplineSegment<double>,
    hardware_interface::EffortJointInterface>
    JointTrajectoryController;

} // namespace inertia_trajectory_controller

PLUGINLIB_EXPORT_CLASS(inertia_trajectory_controller::JointTrajectoryController,
    controller_interface::ControllerBase);
