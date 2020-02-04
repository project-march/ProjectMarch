// Copyright 2019 Project March
#ifndef MARCH_HARDWARE_INTERFACE_MARCH_HARDWARE_INTERFACE_H
#define MARCH_HARDWARE_INTERFACE_MARCH_HARDWARE_INTERFACE_H
#include "march_hardware_interface/march_pdb_state_interface.h"
#include "march_hardware_interface/march_temperature_sensor_interface.h"
#include "march_hardware_interface/power_net_type.h"

#include <memory>
#include <vector>

#include <control_toolbox/filters.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <joint_limits_interface/joint_limits.h>
#include <joint_limits_interface/joint_limits_interface.h>
#include <joint_limits_interface/joint_limits_rosparam.h>
#include <joint_limits_interface/joint_limits_urdf.h>
#include <realtime_tools/realtime_publisher.h>
#include <ros/ros.h>

#include <march_hardware/MarchRobot.h>
#include <march_hardware_builder/hardware_builder.h>
#include <march_shared_resources/AfterLimitJointCommand.h>
#include <march_shared_resources/ImcErrorState.h>

using hardware_interface::JointStateHandle;
using hardware_interface::PositionJointInterface;
using joint_limits_interface::EffortJointSoftLimitsHandle;
using joint_limits_interface::EffortJointSoftLimitsInterface;
using joint_limits_interface::JointLimits;
using joint_limits_interface::PositionJointSoftLimitsHandle;
using joint_limits_interface::PositionJointSoftLimitsInterface;
using joint_limits_interface::SoftJointLimits;

template <typename T>
using RtPublisherPtr = std::unique_ptr<realtime_tools::RealtimePublisher<T>>;

/**
 * @brief HardwareInterface to allow ros_control to actuate our hardware.
 * @details Register an interface for each joint such that they can be actuated
 *     by a controller via ros_control.
 */
class MarchHardwareInterface : public hardware_interface::RobotHW
{
public:
  explicit MarchHardwareInterface(AllowedRobot robotName);

  /**
   * @brief Initialize the HardwareInterface by registering position interfaces
   * for each joint.
   */
  bool init(ros::NodeHandle& nh, ros::NodeHandle& robot_hw_nh) override;

  /**
   * @brief Read actual position from the hardware.
   * @param time Current time
   * @param elapsed_time Duration since last write action
   */
  void read(const ros::Time& time, const ros::Duration& elapsed_time) override;

  /**
   * @brief Perform all safety checks that might crash the exoskeleton.
   */
  void validate();

  /**
   * @brief Write position commands to the hardware.
   * @param time Current time
   * @param elapsed_time Duration since last write action
   */
  void write(const ros::Time& time, const ros::Duration& elapsed_time) override;

private:
  void updatePowerNet();
  void updateHighVoltageEnable();
  void updatePowerDistributionBoard();
  void updateAfterLimitJointCommand();
  void updateIMotionCubeState();
  void initiateIMC();
  void outsideLimitsCheck(size_t joint_index);
  void iMotionCubeStateCheck(size_t joint_index);

  /* March hardware */
  march::MarchRobot march_robot_;
  march::PowerDistributionBoard power_distribution_board_read_;
  PowerNetOnOffCommand power_net_on_off_command_;
  bool has_power_distribution_board_ = false;
  bool master_shutdown_allowed_command_ = false;
  bool enable_high_voltage_command_ = true;

  /* Interfaces */
  hardware_interface::JointStateInterface joint_state_interface_;
  hardware_interface::PositionJointInterface position_joint_interface_;
  hardware_interface::VelocityJointInterface velocity_joint_interface_;
  hardware_interface::EffortJointInterface effort_joint_interface_;

  joint_limits_interface::PositionJointSoftLimitsInterface position_joint_soft_limits_interface_;
  joint_limits_interface::EffortJointSoftLimitsInterface effort_joint_soft_limits_interface_;

  MarchTemperatureSensorInterface march_temperature_interface_;
  MarchPdbStateInterface march_pdb_interface_;

  /* Shared memory */
  size_t num_joints_;
  std::vector<std::string> joint_names_;

  std::vector<double> joint_position_;
  std::vector<double> joint_position_command_;

  std::vector<double> joint_velocity_;
  std::vector<double> joint_velocity_command_;

  std::vector<double> joint_effort_;
  std::vector<double> joint_effort_command_;
  std::vector<double> joint_effort_command_copy;

  std::vector<double> joint_temperature_;
  std::vector<double> joint_temperature_variance_;

  std::vector<SoftJointLimits> soft_limits_;

  /* Real time safe publishers */
  RtPublisherPtr<march_shared_resources::AfterLimitJointCommand> after_limit_joint_command_pub_;
  RtPublisherPtr<march_shared_resources::ImcErrorState> imc_state_pub_;
};

#endif  // MARCH_HARDWARE_INTERFACE_MARCH_HARDWARE_INTERFACE_H
