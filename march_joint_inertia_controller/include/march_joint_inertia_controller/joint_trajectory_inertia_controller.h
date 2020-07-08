// Copyright 2020 Project March.

#ifndef MARCH_HARDWARE_JOINT_TRAJECTORY_INERTIA_CONTROLLER_H
#define MARCH_HARDWARE_JOINT_TRAJECTORY_INERTIA_CONTROLLER_H

#include <control_toolbox/pid.h>
#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <joint_trajectory_controller/joint_trajectory_controller.h>
#include "march_joint_inertia_controller/inertia_estimator.h"
#include <memory>
#include <pluginlib/class_list_macros.h>
#include <realtime_tools/realtime_buffer.h>
#include <realtime_tools/realtime_publisher.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <trajectory_interface/quintic_spline_segment.h>
#include <urdf/model.h>

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
  HardwareInterfaceAdapter() : joint_handles_ptr_(0)
  {
  }

  struct Commands
  {
    double position_;            // Last commanded position
    double velocity_;            // Last commanded velocity
    bool has_velocity_ = false;  // false if no velocity command has been specified
  };

  bool init(std::vector<hardware_interface::JointHandle>& joint_handles, ros::NodeHandle& nh)
  {
    const unsigned int num_joints_ = joint_handles_ptr_->size();

    // Store pointer to joint handles
    joint_handles_ptr_ = &joint_handles;

    // Initialize PIDs
    pids_.resize(joint_handles.size());
    for (unsigned int i = 0; i < pids_.size(); ++i)
    {
      // Node handle to PID gains
      ros::NodeHandle joint_nh(nh, std::string("gains/") + joint_handles[i].getName());

      // Init PID gains from ROS parameter server
      pids_[i].reset(new control_toolbox::Pid());
      if (!pids_[i]->init(joint_nh))
      {
        ROS_WARN_STREAM("Failed to initialize PID gains from ROS parameter server.");
        return false;
      }
    }

    inertia_estimators_.resize(num_joints_);

    // Get joint handle from hardware interface
    for (unsigned int i = 0; i < num_joints_; ++i)
    {
      joints_.push_back((*joint_handles_ptr_)[i]);
      inertia_estimators_.push_back((*joint_handles_ptr_)[i]);
    }

    return true;
  }

  void updateCommand(const ros::Time& /*time*/, const ros::Duration& period,
                     const joint_trajectory_controller::State& /*desired state*/,
                     const joint_trajectory_controller::State& state_error)
  {
    // Preconditions
    if (!joint_handles_ptr_)
      return;
    assert(num_joints_ == state_error.position.size());
    assert(num_joints_ == state_error.velocity.size());

    //  this->fill_buffers(period);
    //  this->inertia_estimate();
    // TO DO: Provide lookup table for gain selection
    // TO DO: apply PID control

    // Update PIDs
    for (unsigned int i = 0; i < num_joints_; ++i)
    {
      const double command = (pids_[i]->computeCommand(state_error.position[i], state_error.velocity[i], period));
      (*joint_handles_ptr_)[i].setCommand(command);
    }
  }

  void starting(const ros::Time& /*time*/)
  {
    std::vector<double> pos_command;
    pos_command.resize(num_joints_);
    for (unsigned int i = 0; i < num_joints_; ++i)
    {
      pos_command[i] = joints_[i].getPosition();

      command_struct_.position_ = pos_command[i];
    }

    command_.initRT(command_struct_);

    // Reset PIDs, zero commands
    for (unsigned int i = 0; i < pids_.size(); ++i)
    {
      pids_[i]->reset();
      (*joint_handles_ptr_)[i].setCommand(0.0);
    }
  }

  void stopping(const ros::Time& /*time*/)
  {
  }

  void setCommand(double pos_command)
  {
    command_struct_.position_ = pos_command;
    command_struct_.has_velocity_ =
        false;  // Flag to ignore the velocity command since our setCommand method did not include it

    // the writeFromNonRT can be used in RT, if you have the guarantee that
    //  * no non-rt thread is calling the same function (we're not subscribing to ros callbacks)
    //  * there is only one single rt thread
    command_.writeFromNonRT(command_struct_);
  }

  void commandCB(const std_msgs::Float64ConstPtr& msg)
  {
    setCommand(msg->data);
  }

  /**
   * \brief Get the PID parameters
   */
  void getGains(int joint_nr, double& p, double& i, double& d, double& i_max, double& i_min)
  {
    bool dummy;
    (*pids_[joint_nr]).getGains(p, i, d, i_max, i_min, dummy);
  }

  /**
   * \brief Get the PID parameters
   */
  void getGains(int joint_nr, double& p, double& i, double& d, double& i_max, double& i_min, bool& antiwindup)
  {
    (*pids_[joint_nr]).getGains(p, i, d, i_max, i_min, antiwindup);
  }

  /**
   * \brief Print debug info to console
   */
  void printDebug(int joint_nr)
  {
    (*pids_[joint_nr]).printValues();
  }

  /**
   * \brief Set the PID parameters
   */
  void setGains(int joint_nr, const double& p, const double& i, const double& d, const double& i_max,
                const double& i_min, const bool& antiwindup = false)
  {
    (*pids_[joint_nr]).setGains(p, i, d, i_max, i_min, antiwindup);
  }

  std::string joint_names;
  realtime_tools::RealtimeBuffer<Commands> command_;
  Commands command_struct_;  // pre-allocated memory that is re-used to set the realtime buffer

private:
  typedef std::shared_ptr<control_toolbox::Pid> PidPtr;
  std::vector<PidPtr> pids_;

  std::vector<hardware_interface::JointHandle>* joint_handles_ptr_;

  unsigned int num_joints_;
  std::vector<hardware_interface::JointHandle> joints_;
  double init_pos_;

  std::vector<InertiaEstimator> inertia_estimators_;
};

#endif  // MARCH_HARDWARE_JOINT_TRAJECTORY_INERTIA_CONTROLLER_H
