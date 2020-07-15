// Copyright 2020 Project March.

#ifndef MARCH_HARDWARE_JOINT_TRAJECTORY_INERTIA_CONTROLLER_H
#define MARCH_HARDWARE_JOINT_TRAJECTORY_INERTIA_CONTROLLER_H

#include <cassert>
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
#include <ros/node_handle.h>
#include <ros/time.h>
#include <trajectory_interface/quintic_spline_segment.h>

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
  HardwareInterfaceAdapter() : joint_handles_ptr_(nullptr)
  {
  }

  /**
   * \brief Initialize the controller by establishing the pointer to the joints, the pid calculators and the inertia
   * estimators
   */
  bool init(std::vector<hardware_interface::JointHandle>& joint_handles, ros::NodeHandle& nh)
  {
    joint_handles_ptr_ = &joint_handles;
    const unsigned int num_joints_ = joint_handles_ptr_->size();

    // Initialize PIDs
    pids_.resize(num_joints_);
    for (unsigned int i = 0; i < pids_.size(); ++i)
    {
      ros::NodeHandle joint_nh(nh, std::string("gains/") + joint_handles[i].getName());

      // Init PID gains from ROS parameter server
      pids_[i].reset(new control_toolbox::Pid());
      if (!pids_[i]->init(joint_nh))
      {
        ROS_WARN_STREAM("Failed to initialize PID gains from ROS parameter server.");
        return false;
      }
    }

    // Obtain inertia estimator parameters from server
    double lambda[2];
    int alpha_filter_size[2];
    ros::NodeHandle rotary_estimator_nh(nh, std::string("inertia_estimator/rotary"));

    if (!rotary_estimator_nh.getParam("/lambda", lambda[0]))
    {
      ROS_ERROR("No lambda specified");
      return false;
    }

    if (!rotary_estimator_nh.getParam("/alpha_filter_size", alpha_filter_size[0]))
    {
      ROS_ERROR("No alpha_filter_size specified");
      return false;
    }

    ros::NodeHandle linear_estimator_nh(nh, std::string("inertia_estimator/linear"));
    if (!linear_estimator_nh.getParam("/lambda", lambda[1]))
    {
      ROS_ERROR("No lambda specified");
      return false;
    }

    if (!linear_estimator_nh.getParam("/alpha_filter_size", alpha_filter_size[1]))
    {
      ROS_ERROR("No alpha_filter_size specified");
      return false;
    }

    // Initialize the estimator parameters
    inertia_estimators_.resize(num_joints_);
    for (unsigned int j = 0; j < num_joints_; ++j)
    {
      inertia_estimators_[j].setNodeHandle(nh);
      inertia_estimators_[j].setLambda(lambda[(int)floor(j / 2) % 2]);  // Produce sequence 00110011
      inertia_estimators_[j].setAcc_size(alpha_filter_size[(int)floor(j / 2) % 2]);
      inertia_estimators_[j].configurePublisher(joint_handles[j].getName());
    }
    return true;
  }

  /**
   * \brief Updates the commanded effort for each individual joint and estimates the inertia on each joint and publishes
   * that information on a topic
   */
  void updateCommand(const ros::Time& /*time*/, const ros::Duration& period,
                     const joint_trajectory_controller::State& /*desired state*/,
                     const joint_trajectory_controller::State& state_error)
  {
    const unsigned int num_joints_ = joint_handles_ptr_->size();

    // Preconditions
    if (!joint_handles_ptr_)
      return;
    assert(num_joints_ == state_error.position.size());
    assert(num_joints_ == state_error.velocity.size());

    // Update inertia estimator
    for (unsigned int j = 0; j < num_joints_; ++j)
    {
      inertia_estimators_[j].fill_buffers((*joint_handles_ptr_)[j].getVelocity(), (*joint_handles_ptr_)[j].getEffort(),
                                          period);
      inertia_estimators_[j].inertia_estimate();
      inertia_estimators_[j].publishInertia();
      // TO DO: Provide lookup table for gain selection
      // TO DO: apply PID control
    }

    // Update PIDs
    for (unsigned int i = 0; i < num_joints_; ++i)
    {
      const double command = (pids_[i]->computeCommand(state_error.position[i], state_error.velocity[i], period));
      (*joint_handles_ptr_)[i].setCommand(command);
    }
  }

  /**
   * \brief Starts the controller by checking if the joint handle pointer is filled, resets the pid calculators and sets
   * the intial command to zero so that the joint doesn't start moving without a desired trajectory
   */
  void starting(const ros::Time& /*time*/)
  {
    if (!joint_handles_ptr_)
    {
      return;
    }

    // Reset PIDs, zero commands
    for (unsigned int i = 0; i < pids_.size(); ++i)
    {
      pids_[i]->reset();
      (*joint_handles_ptr_)[i].setCommand(0.0);
    }
  }

  /**
   * \brief No specific implementation needed I guess, but if necessary later then here it is.
   */
  void stopping(const ros::Time& /*time*/)
  {
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
   * \brief Get the PID parameters with specification if anti windup is used
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
