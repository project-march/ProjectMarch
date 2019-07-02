#ifndef ROS_CONTROL__MARCH_HARDWARE_INTERFACE_H
#define ROS_CONTROL__MARCH_HARDWARE_INTERFACE_H

#include <control_toolbox/filters.h>
#include <march_hardware_interface/march_hardware.h>
#include <ros/ros.h>

#include <march_hardware_builder/HardwareBuilder.h>

#include <march_hardware/MarchRobot.h>

using namespace hardware_interface;
using joint_limits_interface::JointLimits;
using joint_limits_interface::SoftJointLimits;
using joint_limits_interface::PositionJointSoftLimitsHandle;
using joint_limits_interface::PositionJointSoftLimitsInterface;

namespace march_hardware_interface {
static const double POSITION_STEP_FACTOR = 10;
static const double VELOCITY_STEP_FACTOR = 10;

/**
 * @brief HardwareInterface to allow ros_control to actuate our hardware.
 * @details Register an interface for each joint such that they can be actuated
 *     by a controller via ros_control.
 */
class MarchHardwareInterface : public march_hardware_interface::MarchHardware {
public:
  MarchHardwareInterface(ros::NodeHandle &nh, AllowedRobot robotName);
  ~MarchHardwareInterface();

  /**
   * @brief Initialize the HardwareInterface by registering position interfaces
   * for each joint.
   */
  void init();
  void update(const ros::TimerEvent &e);

  /**
   * @brief Read actual postion from the hardware.
   */
  void read(ros::Duration elapsed_time = ros::Duration(0.01));

  /**
   * @brief Write position commands to the hardware.
   * @param elapsed_time Duration since last write action
   */
  void write(ros::Duration elapsed_time);

protected:
  ::march4cpp::MarchRobot marchRobot;
  ros::NodeHandle nh_;
  ros::Timer non_realtime_loop_;
  ros::Duration control_period_;
  ros::Duration elapsed_time_;
  PositionJointInterface positionJointInterface;
  PositionJointSoftLimitsInterface positionJointSoftLimitsInterface;
  double loop_hz_;
  bool hasPowerDistributionBoard = false;
  boost::shared_ptr<controller_manager::ControllerManager> controller_manager_;
  double p_error_, v_error_, e_error_;

private:
  void updatePowerNet();
  void updateHighVoltageEnable();
  void updatePowerDistributionBoard();
};
}

#endif