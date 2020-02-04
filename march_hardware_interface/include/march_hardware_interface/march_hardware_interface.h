// Copyright 2019 Project March
#ifndef MARCH_HARDWARE_INTERFACE_MARCH_HARDWARE_INTERFACE_H
#define MARCH_HARDWARE_INTERFACE_MARCH_HARDWARE_INTERFACE_H
#include <vector>

#include <control_toolbox/filters.h>
#include <march_hardware_interface/march_hardware.h>
#include <ros/ros.h>
#include <realtime_tools/realtime_publisher.h>
#include <march_shared_resources/ImcErrorState.h>
#include <march_shared_resources/AfterLimitJointCommand.h>
#include <march_hardware_builder/hardware_builder.h>

#include <march_hardware/MarchRobot.h>

using hardware_interface::JointStateHandle;
using hardware_interface::PositionJointInterface;
using joint_limits_interface::EffortJointSoftLimitsHandle;
using joint_limits_interface::EffortJointSoftLimitsInterface;
using joint_limits_interface::JointLimits;
using joint_limits_interface::PositionJointSoftLimitsHandle;
using joint_limits_interface::PositionJointSoftLimitsInterface;
using joint_limits_interface::SoftJointLimits;

/**
 * @brief HardwareInterface to allow ros_control to actuate our hardware.
 * @details Register an interface for each joint such that they can be actuated
 *     by a controller via ros_control.
 */
class MarchHardwareInterface : public MarchHardware
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

protected:
  ::march::MarchRobot marchRobot;
  PositionJointSoftLimitsInterface positionJointSoftLimitsInterface;
  EffortJointSoftLimitsInterface effortJointSoftLimitsInterface;
  bool hasPowerDistributionBoard = false;
  typedef boost::shared_ptr<realtime_tools::RealtimePublisher<march_shared_resources::ImcErrorState> > RtPublisherPtr;
  typedef boost::shared_ptr<realtime_tools::RealtimePublisher<march_shared_resources::AfterLimitJointCommand> >
      RtPublisherAfterLimitJointCommandPtr;
  RtPublisherAfterLimitJointCommandPtr after_limit_joint_command_pub_;
  RtPublisherPtr imc_state_pub_;
  std::vector<SoftJointLimits> soft_limits_;

private:
  void updatePowerNet();
  void updateHighVoltageEnable();
  void updatePowerDistributionBoard();
  void updateAfterLimitJointCommand();
  void updateIMotionCubeState();
  void initiateIMC();
  void outsideLimitsCheck(int joint_index);
  void iMotionCubeStateCheck(int joint_index);
};

#endif  // MARCH_HARDWARE_INTERFACE_MARCH_HARDWARE_INTERFACE_H
