#ifndef MARCH_HARDWARE_TRAJECTORY_MPC_H
#define MARCH_HARDWARE_TRAJECTORY_MPC_H

// ROS Control includes
#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <joint_trajectory_controller/joint_trajectory_controller.h>
#include <joint_trajectory_controller/trajectory_interface/quintic_spline_segment.h>
#include "model_predictive_controller.hpp"

// Other includes
#include <ros/ros.h>
#include <pluginlib/class_list_macros.h>
#include <vector>

// Publishing and msgs
#include <ros/node_handle.h>
#include <ros/time.h>
#include <march_shared_msgs/MpcMsg.h>
#include <realtime_tools/realtime_publisher.h>

#include <cassert>
#include <memory>
#include <time.h>

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
                     const std::vector<joint_trajectory_controller::State>&  /*desired_states*/,
                     const joint_trajectory_controller::State& state_error);

  /**
   * \brief Procedure, if required, for stopping the controller
   */
  void stopping(const ros::Time& /*time*/);
  /**
   * \brief Start MPC topic and resize vectors in the MPC msg.
   */
  void initMpcMsg();
  /**
   * \brief Set and publish the message with MPC outputs, and MPC configuration
   */
  void setMpcMsg(int joint_number);

private:
  /**
   * @brief Retrieve the Q matrix from the parameter server for a joint.
   * @param joint_name Joint to retrieve Q Matrix for
   * @return Returns a 2d vector: The Q Matrix.
   */
  std::vector<std::vector<float>> getQMatrix(std::string joint_name);

  std::vector<hardware_interface::JointHandle>* joint_handles_ptr_;

  unsigned int num_joints_;
  double command;

  std::vector<ModelPredictiveController> model_predictive_controllers_;
  vector<double> state;

  std::unique_ptr<realtime_tools::RealtimePublisher<march_shared_msgs::MpcMsg>> mpc_pub_;
};

// Assign an alias to the class definition
typedef HardwareInterfaceAdapter<hardware_interface::EffortJointInterface, joint_trajectory_controller::State> ModelPredictiveControllerInterface;

#endif  // MARCH_HARDWARE_TRAJECTORY_MPC_H