#ifndef MARCH_HARDWARE_TRAJECTORY_MPC_H
#define MARCH_HARDWARE_TRAJECTORY_MPC_H

// ROS Control includes
#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <joint_trajectory_controller/joint_trajectory_controller.h>
#include <trajectory_interface/quintic_spline_segment.h>

// Other includes
#include <ros/ros.h>
#include <pluginlib/class_list_macros.h>


// Create a State alias
namespace joint_trajectory_controller
{
typedef trajectory_interface::QuinticSplineSegment<double> SegmentImpl;
typedef JointTrajectorySegment<SegmentImpl> Segment;
typedef typename Segment::State State;
}  // namespace joint_trajectory_controller

// Create the HardwareInterfaceAdapter class that handles the initializing, starting, updating and stopping of the controller
template <>
class HardwareInterfaceAdapter<hardware_interface::EffortJointInterface, joint_trajectory_controller::State>
{
public:
  HardwareInterfaceAdapter() : joint_handles_ptr_(nullptr) { }

  /**
   * \brief Initialize the controller by establishing the pointer to the joints
   */
  bool init(std::vector<hardware_interface::JointHandle>& joint_handles, ros::NodeHandle& nh);

  /**
  * \brief Starts the controller by checking if the joint handle pointer is filled and sets
  * the initial command to zero so that the joint doesn't start moving without a desired trajectory
  */
  void starting(const ros::Time& /*time*/);

  /**
   * \brief Updates the commanded effort for each individual joint and estimates the inertia on each joint and publishes
   * that information on a topic
   */
  void updateCommand(const ros::Time& /*time*/, const ros::Duration& period,
                     const joint_trajectory_controller::State& /*desired state*/,
                     const joint_trajectory_controller::State& state_error);

  /**
   * \brief Procedure, if required, for stopping the controller
   */
  void stopping(const ros::Time& /*time*/);

private:
  std::vector<hardware_interface::JointHandle>* joint_handles_ptr_;

  unsigned int num_joints_;

};

// Assign an alias to the class definition
typedef HardwareInterfaceAdapter<hardware_interface::EffortJointInterface, joint_trajectory_controller::State> ModelPredictiveControllerInterface;

#endif  // MARCH_HARDWARE_TRAJECTORY_MPC_H