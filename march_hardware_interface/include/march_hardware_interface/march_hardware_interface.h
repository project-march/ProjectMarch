#ifndef ROS_CONTROL__MARCH_HARDWARE_INTERFACE_H
#define ROS_CONTROL__MARCH_HARDWARE_INTERFACE_H

#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <joint_limits_interface/joint_limits_interface.h>
#include <joint_limits_interface/joint_limits.h>
#include <joint_limits_interface/joint_limits_urdf.h>
#include <joint_limits_interface/joint_limits_rosparam.h>
#include <controller_manager/controller_manager.h>
#include <boost/scoped_ptr.hpp>
#include <ros/ros.h>
#include <march_hardware_interface/march_hardware.h>
#include <march_hardware/March4.h>

using namespace hardware_interface;
using joint_limits_interface::JointLimits;
using joint_limits_interface::SoftJointLimits;
using joint_limits_interface::PositionJointSoftLimitsHandle;
using joint_limits_interface::PositionJointSoftLimitsInterface;

namespace march_hardware_interface
{
static const double POSITION_STEP_FACTOR = 10;
static const double VELOCITY_STEP_FACTOR = 10;

/**
 * @brief HardwareInterface to allow ros_control to actuate our hardware.
 * @details Register an interface for each joint such that they can be actuated
 *     by a controller via ros_control.
 */
class MarchHardwareInterface : public march_hardware_interface::MarchHardware
{
public:
  MarchHardwareInterface(ros::NodeHandle& nh);
  ~MarchHardwareInterface();

  /**
   * @brief Initialize the HardwareInterface by registering position interfaces for each joint.
   */
  void init();
  void update(const ros::TimerEvent& e);

  /**
   * @brief Read actual postion from the hardware.
   */
  void read();

  /**
   * @brief Write position commands to the hardware.
   * @param elapsed_time Duration since last write action
   */
  void write(ros::Duration elapsed_time);

protected:
  ::march4cpp::MARCH4 march;
  ros::NodeHandle nh_;
  ros::Timer non_realtime_loop_;
  ros::Duration control_period_;
  ros::Duration elapsed_time_;
  PositionJointInterface positionJointInterface;
  PositionJointSoftLimitsInterface positionJointSoftLimitsInterface;
  double loop_hz_;
  boost::shared_ptr<controller_manager::ControllerManager> controller_manager_;
  double p_error_, v_error_, e_error_;
};
}

#endif