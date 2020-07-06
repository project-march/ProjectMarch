// Copyright 2020 Project March.

#ifndef MARCH_HARDWARE_JOINT_INERTIA_CONTROLLER_H
#define MARCH_HARDWARE_JOINT_INERTIA_CONTROLLER_H

#include <control_toolbox/pid.h>
#include <controller_interface/controller.h>
#include <trajectory_interface/quintic_spline_segment.h>
#include <joint_trajectory_controller/joint_trajectory_controller.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <memory>
#include <urdf/model.h>
#include <realtime_tools/realtime_buffer.h>
#include <realtime_tools/realtime_publisher.h>
#include <pluginlib/class_list_macros.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Float64.h>

namespace joint_trajectory_controller
{
typedef trajectory_interface::QuinticSplineSegment<double> SegmentImpl;
typedef JointTrajectorySegment<SegmentImpl> Segment;
typedef typename Segment::State State;
}  // namespace joint_trajectory_controller

template <>
class HardwareInterfaceAdapter<hardware_interface::EffortJointInterface, joint_trajectory_controller::State>
{
public:
  struct Commands
  {
    double position_;            // Last commanded position
    double velocity_;            // Last commanded velocity
    bool has_velocity_ = false;  // false if no velocity command has been specified
  };

  bool init(hardware_interface::PositionJointInterface* hw, ros::NodeHandle& n);
  void updateCommand(const ros::Time& time, const ros::Duration& period,
                     const joint_trajectory_controller::State& desired_state,
                     const joint_trajectory_controller::State& state_error);
  void starting(const ros::Time& time);
  void stopping(const ros::Time& time);
  void setCommand(double pos_command);
  void setCommand(double pos_command, double vel_command);
  void commandCB(const std_msgs::Float64ConstPtr& msg);

  /**
   * \brief Get the PID parameters
   */
  void getGains(double& p, double& i, double& d, double& i_max, double& i_min);
  /**
   * \brief Get the PID parameters
   */
  void getGains(double& p, double& i, double& d, double& i_max, double& i_min, bool& antiwindup);

  /**
   * \brief Print debug info to console
   */
  void printDebug();

  /**
   * \brief Set the PID parameters
   */
  void setGains(const double& p, const double& i, const double& d, const double& i_max, const double& i_min,
                const bool& antiwindup = false);

  double getPosition();

  std::string getJointName();

  urdf::JointConstSharedPtr joint_urdf_;
  std::string joint_name;
  realtime_tools::RealtimeBuffer<Commands> command_;
  Commands command_struct_;  // pre-allocated memory that is re-used to set the realtime buffer

private:
  int loop_count_;
  control_toolbox::Pid pid_controller_; /**< Internal PID controller. */

  ros::Subscriber sub_command_;
  hardware_interface::JointHandle joint_;
  double init_pos_;

  std::vector<InertiaEstimator> inertia_estimators;

  /**
   * \brief Callback from /command subscriber for setpoint
   */
  void setCommandCB(const std_msgs::Float64ConstPtr& msg);
};
}  // namespace joint_inertia_controller

#endif  // MARCH_HARDWARE_JOINT_INERTIA_CONTROLLER_H
