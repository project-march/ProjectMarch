// Copyright 2019 Project March.
#include "march_hardware_interface/march_hardware_interface.h"
#include "march_hardware_interface/power_net_on_off_command.h"

#include <memory>
#include <sstream>
#include <string>

#include <joint_limits_interface/joint_limits.h>
#include <joint_limits_interface/joint_limits_interface.h>
#include <joint_limits_interface/joint_limits_urdf.h>
#include <urdf/model.h>

#include <march_hardware/ActuationMode.h>
#include <march_hardware/Joint.h>

using hardware_interface::JointHandle;
using hardware_interface::JointStateHandle;
using hardware_interface::PositionJointInterface;
using joint_limits_interface::EffortJointSoftLimitsHandle;
using joint_limits_interface::JointLimits;
using joint_limits_interface::PositionJointSoftLimitsHandle;
using joint_limits_interface::SoftJointLimits;
using march::Joint;

MarchHardwareInterface::MarchHardwareInterface(std::unique_ptr<march::MarchRobot> robot)
  : march_robot_(std::move(robot))
  , has_power_distribution_board_(this->march_robot_->getPowerDistributionBoard().getSlaveIndex() != -1)
{
  // Get joint names from urdf
  for (const auto& urdf_joint : this->march_robot_->getUrdf().joints_)
  {
    if (urdf_joint.second->type != urdf::Joint::FIXED)
    {
      this->joint_names_.push_back(urdf_joint.first);
    }
  }

  this->num_joints_ = this->joint_names_.size();
}

bool MarchHardwareInterface::init(ros::NodeHandle& nh, ros::NodeHandle& /* robot_hw_nh */)
{
  // Initialize realtime publisher for the IMotionCube states
  this->imc_state_pub_ = std::make_unique<realtime_tools::RealtimePublisher<march_shared_resources::ImcErrorState>>(
      nh, "/march/imc_states/", 4);

  this->after_limit_joint_command_pub_ =
      std::make_unique<realtime_tools::RealtimePublisher<march_shared_resources::AfterLimitJointCommand>>(
          nh, "/march/controller/after_limit_joint_command/", 4);

  nh.setParam("/march/joint_names", this->joint_names_);

  this->reserveMemory();

  // Start ethercat cycle in the hardware
  this->march_robot_->startEtherCAT();

  for (size_t i = 0; i < num_joints_; ++i)
  {
    SoftJointLimits soft_limits;
    getSoftJointLimits(this->march_robot_->getUrdf().getJoint(joint_names_[i]), soft_limits);
    ROS_DEBUG("[%s] Soft limits set to (%f, %f)", joint_names_[i].c_str(), soft_limits.min_position,
              soft_limits.max_position);
    soft_limits_[i] = soft_limits;
  }

  // Create march_pdb_state interface
  MarchPdbStateHandle march_pdb_state_handle("PDBhandle", &power_distribution_board_read_,
                                             &master_shutdown_allowed_command_, &enable_high_voltage_command_,
                                             &power_net_on_off_command_);
  march_pdb_interface_.registerHandle(march_pdb_state_handle);

  registerInterface(&march_temperature_interface_);
  registerInterface(&march_pdb_interface_);
  registerInterface(&joint_state_interface_);
  registerInterface(&position_joint_interface_);
  registerInterface(&effort_joint_interface_);
  registerInterface(&position_joint_soft_limits_interface_);
  registerInterface(&effort_joint_soft_limits_interface_);

  if (has_power_distribution_board_)
  {
    for (size_t i = 0; i < num_joints_; i++)
    {
      int net_number = march_robot_->getJoint(joint_names_[i]).getNetNumber();
      if (net_number == -1)
      {
        std::ostringstream error_stream;
        error_stream << "Joint " << joint_names_[i].c_str() << " has no net number";
        throw std::runtime_error(error_stream.str());
      }
      while (!march_robot_->getPowerDistributionBoard().getHighVoltage().getNetOperational(net_number))
      {
        march_robot_->getPowerDistributionBoard().getHighVoltage().setNetOnOff(true, net_number);
        usleep(100000);
        ROS_WARN("[%s] Waiting on high voltage", joint_names_[i].c_str());
      }
    }
  }
  else
  {
    ROS_WARN("Running without Power Distribution Board");
  }

  // Initialize interfaces for each joint
  for (size_t i = 0; i < num_joints_; ++i)
  {
    march::Joint joint = march_robot_->getJoint(joint_names_[i]);
    // Create joint state interface
    JointStateHandle joint_state_handle(joint.getName(), &joint_position_[i], &joint_velocity_[i], &joint_effort_[i]);
    joint_state_interface_.registerHandle(joint_state_handle);

    // Retrieve joint (soft) limits from the urdf
    JointLimits limits;
    getJointLimits(this->march_robot_->getUrdf().getJoint(joint.getName()), limits);

    if (joint.getActuationMode() == march::ActuationMode::position)
    {
      // Create position joint interface
      JointHandle joint_position_handle(joint_state_handle, &joint_position_command_[i]);
      position_joint_interface_.registerHandle(joint_position_handle);

      // Create joint limit interface
      PositionJointSoftLimitsHandle joint_limits_handle(joint_position_handle, limits, soft_limits_[i]);
      position_joint_soft_limits_interface_.registerHandle(joint_limits_handle);
    }
    else if (joint.getActuationMode() == march::ActuationMode::torque)
    {
      // Create effort joint interface
      JointHandle joint_effort_handle_(joint_state_handle, &joint_effort_command_[i]);
      effort_joint_interface_.registerHandle(joint_effort_handle_);

      // Create joint effort limit interface
      EffortJointSoftLimitsHandle joint_effort_limits_handle(joint_effort_handle_, limits, soft_limits_[i]);
      effort_joint_soft_limits_interface_.registerHandle(joint_effort_limits_handle);
    }

    // Create velocity joint interface
    JointHandle joint_velocity_handle(joint_state_handle, &joint_velocity_command_[i]);
    velocity_joint_interface_.registerHandle(joint_velocity_handle);

    // Create march_state interface
    MarchTemperatureSensorHandle temperature_sensor_handle(joint_names_[i], &joint_temperature_[i],
                                                           &joint_temperature_variance_[i]);
    march_temperature_interface_.registerHandle(temperature_sensor_handle);

    // Enable high voltage on the IMC
    if (joint.canActuate())
    {
      if (has_power_distribution_board_)
      {
        int net_number = joint.getNetNumber();
        if (net_number != -1)
        {
          march_robot_->getPowerDistributionBoard().getHighVoltage().setNetOnOff(true, net_number);
        }
        else
        {
          ROS_FATAL("Joint %s has no high voltage net number", joint.getName().c_str());
          return false;
        }
      }
      joint.prepareActuation();

      // Set the first target as the current position
      joint_position_[i] = joint.getAngleRadAbsolute();
      incremental_joint_position_[i] = joint.getAngleRadMostPrecise();
      joint_velocity_[i] = 0;
      joint_effort_[i] = 0;

      if (joint.getActuationMode() == march::ActuationMode::position)
      {
        joint_position_command_[i] = joint_position_[i];
      }
      else if (joint.getActuationMode() == march::ActuationMode::torque)
      {
        joint_effort_command_[i] = 0;
      }
    }
  }
  ROS_INFO("Successfully actuated all joints");
  return true;
}

void MarchHardwareInterface::validate()
{
  for (size_t i = 0; i < num_joints_; i++)
  {
    this->outsideLimitsCheck(i);
    this->iMotionCubeStateCheck(i);
  }
}

void MarchHardwareInterface::read(const ros::Time& /* time */, const ros::Duration& elapsed_time)
{
  for (size_t i = 0; i < num_joints_; i++)
  {
    march::Joint joint = march_robot_->getJoint(joint_names_[i]);
    new_absolute_joint_position = joint.getAngleRadAbsolute();
    new_incremental_joint_position = joint.getAngleRadIncremental();

    // Get velocity from encoder position
    const double incremental_joint_velocity = (new_incremental_joint_position - incremental_joint_position_[i]) / elapsed_time.toSec();
    const double absolute_joint_velocity = (new_absolute_joint_position - joint_position_[i]) / elapsed_time.toSec();
    const double incremental_joint_acceleration = (incremental_joint_velocity - joint_velocity[i]) / elapsed_time.toSec();
    const double absolute_joint_acceleration = (absolute_joint_velocity - joint_velocity[i]) / elapsed_time.toSec();

    if (std::abs(incremental_joint_velocity - absolute_joint_velocity) > 2 * (joint.getAbsoluteResolution + joint.getIncrementalResolution)) {
        // Take the velocity that is closest to that of the previous timestep.
        if (std::abs(incremental_joint_velocity - joint_velocity[i]) < std::abs(absolute_joint_velocity - joint_velocity[i])) {
            joint_velocity[i] = incremental_joint_velocity;
        }
        else {
            joint_velocity[i] = absolute_joint_velocity;
        }
    }
    else {
        // Take the velocity with the highest resolution.
        if (joint.is_incremental_more_precise()) {
            joint_velocity[i] = incremental_joint_velocity;
        }
        else {
            joint_velocity[i] = absolute_joint_velocity;
        }
    }

    // Update position with he most accurate velocity
    joint_position[i] += joint_velocity[i] * elapsed_time.toSec();





    if (std::abs(incremental_joint_acceleration) > max_acceleration[i] &&
        std::abs(absolute_joint_acceleration) > max_acceleration[i]) {

    }
    else if (std::abs(incremental_joint_acceleration) > max_acceleration[i]) {

    }
    else if (std::abs(absolute_joint_acceleration) > max_acceleration[i]) {

    }
    else {
        if (joint.is_incremental_more_precise()) {

        }
    }



    if (std::abs(incremental_joint_acceleration) < max_acceleration[i]) {
        joint_velocity[i] = incremental_joint_velocity;
        incremental_joint_position_[i]= new_incremental_joint_position;
    }
    else {
        outlier_incremental[i] = true;

        if (std::abs(absolute_joint_acceleration) < max_acceleration[i]) {
            joint_velocity[i] = absolute_joint_velocity;
        }

        incremental_joint_position_[i] += joint_velocity[i] * elapsed_time.toSec();
    }

      if (std::abs(absolute_joint_acceleration) < max_acceleration[i]) {
          joint_position[i] = new_absolute_joint_position;
      }
      else {
          joint_position[i] += joint_velocity[i];
      }








      if (joint.hasTemperatureGES())
      {
          joint_temperature_[i] = joint.getTemperature();
      }
    joint_effort_[i] = joint.getTorque();
  }

  this->updateIMotionCubeState();

  if (has_power_distribution_board_)
  {
    power_distribution_board_read_ = march_robot_->getPowerDistributionBoard();
  }
}

void MarchHardwareInterface::write(const ros::Time& /* time */, const ros::Duration& elapsed_time)
{
  for (size_t i = 0; i < num_joints_; i++)
  {
    // Enlarge joint_effort_command because ROS control limits the pid values to a certain maximum
    joint_effort_command_[i] = joint_effort_command_[i] * 1000.0;
  }

  // Enforce limits on all joints in effort mode
  effort_joint_soft_limits_interface_.enforceLimits(elapsed_time);

  // Enforce limits on all joints in position mode
  position_joint_soft_limits_interface_.enforceLimits(elapsed_time);

  for (size_t i = 0; i < num_joints_; i++)
  {
    march::Joint joint = march_robot_->getJoint(joint_names_[i]);

    if (joint.canActuate())
    {
      if (joint.getActuationMode() == march::ActuationMode::position)
      {
        joint.actuateRad(joint_position_command_[i]);
      }
      else if (joint.getActuationMode() == march::ActuationMode::torque)
      {
        joint.actuateTorque(std::round(joint_effort_command_[i]));
      }
    }
  }

  this->updateAfterLimitJointCommand();

  if (has_power_distribution_board_)
  {
    updatePowerDistributionBoard();
  }
}

int MarchHardwareInterface::getEthercatCycleTime() const
{
  return this->march_robot_->getEthercatCycleTime();
}

void MarchHardwareInterface::reserveMemory()
{
  joint_position_.resize(num_joints_);
  incremental_joint_position_.resize(num_joints_);
  joint_position_command_.resize(num_joints_);
  joint_velocity_.resize(num_joints_);
  joint_velocity_command_.resize(num_joints_);
  joint_effort_.resize(num_joints_);
  joint_effort_command_.resize(num_joints_);
  joint_temperature_.resize(num_joints_);
  joint_temperature_variance_.resize(num_joints_);
  soft_limits_.resize(num_joints_);

  after_limit_joint_command_pub_->msg_.name.resize(num_joints_);
  after_limit_joint_command_pub_->msg_.position_command.resize(num_joints_);
  after_limit_joint_command_pub_->msg_.effort_command.resize(num_joints_);

  imc_state_pub_->msg_.joint_names.resize(num_joints_);
  imc_state_pub_->msg_.status_word.resize(num_joints_);
  imc_state_pub_->msg_.detailed_error.resize(num_joints_);
  imc_state_pub_->msg_.motion_error.resize(num_joints_);
  imc_state_pub_->msg_.state.resize(num_joints_);
  imc_state_pub_->msg_.detailed_error_description.resize(num_joints_);
  imc_state_pub_->msg_.motion_error_description.resize(num_joints_);
  imc_state_pub_->msg_.motor_current.resize(num_joints_);
  imc_state_pub_->msg_.motor_voltage.resize(num_joints_);
  imc_state_pub_->msg_.incremental_encoder_value.resize(num_joints_);
}

void MarchHardwareInterface::updatePowerDistributionBoard()
{
  march_robot_->getPowerDistributionBoard().setMasterOnline();
  march_robot_->getPowerDistributionBoard().setMasterShutDownAllowed(master_shutdown_allowed_command_);
  updateHighVoltageEnable();
  updatePowerNet();
}

void MarchHardwareInterface::updateHighVoltageEnable()
{
  try
  {
    if (march_robot_->getPowerDistributionBoard().getHighVoltage().getHighVoltageEnabled() !=
        enable_high_voltage_command_)
    {
      march_robot_->getPowerDistributionBoard().getHighVoltage().enableDisableHighVoltage(enable_high_voltage_command_);
    }
    else if (!march_robot_->getPowerDistributionBoard().getHighVoltage().getHighVoltageEnabled())
    {
      ROS_WARN_THROTTLE(2, "High voltage disabled");
    }
  }
  catch (std::exception& exception)
  {
    ROS_ERROR("%s", exception.what());
    ROS_DEBUG("Reverting the enable_high_voltage_command input, in attempt to prevent this exception is thrown "
              "again");
    enable_high_voltage_command_ = !enable_high_voltage_command_;
  }
}

void MarchHardwareInterface::updatePowerNet()
{
  if (power_net_on_off_command_.getType() == PowerNetType::high_voltage)
  {
    try
    {
      if (march_robot_->getPowerDistributionBoard().getHighVoltage().getNetOperational(
              power_net_on_off_command_.getNetNumber()) != power_net_on_off_command_.isOnOrOff())
      {
        march_robot_->getPowerDistributionBoard().getHighVoltage().setNetOnOff(
            power_net_on_off_command_.isOnOrOff(), power_net_on_off_command_.getNetNumber());
      }
    }
    catch (std::exception& exception)
    {
      ROS_ERROR("%s", exception.what());
      ROS_DEBUG("Reset power net command, in attempt to prevent this exception is thrown again");
      power_net_on_off_command_.reset();
    }
  }
  else if (power_net_on_off_command_.getType() == PowerNetType::low_voltage)
  {
    try
    {
      if (march_robot_->getPowerDistributionBoard().getLowVoltage().getNetOperational(
              power_net_on_off_command_.getNetNumber()) != power_net_on_off_command_.isOnOrOff())
      {
        march_robot_->getPowerDistributionBoard().getLowVoltage().setNetOnOff(power_net_on_off_command_.isOnOrOff(),
                                                                              power_net_on_off_command_.getNetNumber());
      }
    }
    catch (std::exception& exception)
    {
      ROS_ERROR("%s", exception.what());
      ROS_WARN("Reset power net command, in attempt to prevent this exception is thrown again");
      power_net_on_off_command_.reset();
    }
  }
}

void MarchHardwareInterface::updateAfterLimitJointCommand()
{
  if (!after_limit_joint_command_pub_->trylock())
  {
    return;
  }

  after_limit_joint_command_pub_->msg_.header.stamp = ros::Time::now();
  for (size_t i = 0; i < num_joints_; i++)
  {
    march::Joint joint = march_robot_->getJoint(joint_names_[i]);

    after_limit_joint_command_pub_->msg_.name[i] = joint.getName();
    after_limit_joint_command_pub_->msg_.position_command[i] = joint_position_command_[i];
    after_limit_joint_command_pub_->msg_.effort_command[i] = joint_effort_command_[i];
  }

  after_limit_joint_command_pub_->unlockAndPublish();
}

void MarchHardwareInterface::updateIMotionCubeState()
{
  if (!imc_state_pub_->trylock())
  {
    return;
  }

  imc_state_pub_->msg_.header.stamp = ros::Time::now();
  for (size_t i = 0; i < num_joints_; i++)
  {
    march::IMotionCubeState imc_state = march_robot_->getJoint(joint_names_[i]).getIMotionCubeState();
    imc_state_pub_->msg_.header.stamp = ros::Time::now();
    imc_state_pub_->msg_.joint_names[i] = joint_names_[i];
    imc_state_pub_->msg_.status_word[i] = imc_state.statusWord;
    imc_state_pub_->msg_.detailed_error[i] = imc_state.detailedError;
    imc_state_pub_->msg_.motion_error[i] = imc_state.motionError;
    imc_state_pub_->msg_.state[i] = imc_state.state.getString();
    imc_state_pub_->msg_.detailed_error_description[i] = imc_state.detailedErrorDescription;
    imc_state_pub_->msg_.motion_error_description[i] = imc_state.motionErrorDescription;
    imc_state_pub_->msg_.motor_current[i] = imc_state.motorCurrent;
    imc_state_pub_->msg_.motor_voltage[i] = imc_state.motorVoltage;
    imc_state_pub_->msg_.incremental_encoder_value[i] = imc_state.incrementalEncoderValue;
  }

  imc_state_pub_->unlockAndPublish();
}

void MarchHardwareInterface::iMotionCubeStateCheck(size_t joint_index)
{
  march::IMotionCubeState imc_state = march_robot_->getJoint(joint_names_[joint_index]).getIMotionCubeState();
  if (imc_state.state == march::IMCState::FAULT)
  {
    this->march_robot_->stopEtherCAT();
    std::ostringstream error_stream;
    error_stream << "IMotionCube of joint " << joint_names_[joint_index].c_str() << " is in fault state "
                 << imc_state.state.getString() << std::endl;
    error_stream << "Detailed Error: " << imc_state.detailedErrorDescription << "(" << imc_state.detailedError << ")"
                 << std::endl;
    error_stream << "Motion Error: " << imc_state.motionErrorDescription << "(" << imc_state.motionError << ")"
                 << std::endl;

    throw std::runtime_error(error_stream.str());
  }
}

void MarchHardwareInterface::outsideLimitsCheck(size_t joint_index)
{
  march::Joint joint = march_robot_->getJoint(joint_names_[joint_index]);
  if (joint_position_[joint_index] < soft_limits_[joint_index].min_position ||
      joint_position_[joint_index] > soft_limits_[joint_index].max_position)
  {
    ROS_ERROR_THROTTLE(1, "Joint %s is outside of its soft limits (%f, %f). Actual position: %f",
                       joint_names_[joint_index].c_str(), soft_limits_[joint_index].min_position,
                       soft_limits_[joint_index].max_position, joint_position_[joint_index]);

    if (joint.canActuate())
    {
      std::ostringstream error_stream;
      error_stream << "Joint " << joint_names_[joint_index].c_str() << " is out of its soft limits ("
                   << soft_limits_[joint_index].min_position << ", " << soft_limits_[joint_index].max_position
                   << "). Actual position: " << joint_position_[joint_index];
      throw std::runtime_error(error_stream.str());
    }
  }
}
