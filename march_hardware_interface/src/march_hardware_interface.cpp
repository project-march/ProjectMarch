// Copyright 2019 Project March.

#include <joint_limits_interface/joint_limits.h>
#include <joint_limits_interface/joint_limits_interface.h>
#include <joint_limits_interface/joint_limits_urdf.h>
#include <sstream>
#include <string>

#include <march_hardware/MarchRobot.h>
#include <march_hardware/Joint.h>
#include <march_hardware/ActuationMode.h>

#include <march_hardware_interface/power_net_on_off_command.h>
#include <march_hardware_interface/march_hardware_interface.h>

#include <urdf/model.h>

using hardware_interface::JointHandle;
using hardware_interface::JointStateHandle;
using hardware_interface::PositionJointInterface;
using joint_limits_interface::EffortJointSoftLimitsHandle;
using joint_limits_interface::EffortJointSoftLimitsInterface;
using joint_limits_interface::JointLimits;
using joint_limits_interface::PositionJointSoftLimitsHandle;
using joint_limits_interface::PositionJointSoftLimitsInterface;
using joint_limits_interface::SoftJointLimits;
using march::Joint;

MarchHardwareInterface::MarchHardwareInterface(ros::NodeHandle& nh, AllowedRobot robotName)
  : nh_(nh), marchRobot(HardwareBuilder(robotName).createMarchRobot())
{
}

void MarchHardwareInterface::init()
{
  // Initialize realtime publisher for the IMotionCube states
  imc_state_pub_ = RtPublisherPtr(
      new realtime_tools::RealtimePublisher<march_shared_resources::ImcErrorState>(this->nh_, "/march/imc_states/", 4));

  after_limit_joint_command_pub_ = RtPublisherAfterLimitJointCommandPtr(
      new realtime_tools::RealtimePublisher<march_shared_resources::AfterLimitJointCommand>(
          this->nh_, "/march/controller/after_limit_joint_command/", 4));

  // Start ethercat cycle in the hardware
  this->marchRobot.startEtherCAT();

  urdf::Model model;
  if (!model.initParam("/robot_description"))
  {
    ROS_ERROR("Failed to read the urdf from the parameter server.");
    throw std::runtime_error("Failed to read the urdf from the parameter server.");
  }

  // Get joint names from urdf
  for (auto const& urdfJoint : model.joints_)
  {
    if (urdfJoint.second->type != urdf::Joint::FIXED)
    {
      joint_names_.push_back(urdfJoint.first);
    }
  }

  nh_.setParam("/march/joint_names", joint_names_);

  num_joints_ = joint_names_.size();

  // Resize vectors
  joint_position_.resize(num_joints_);
  joint_velocity_.resize(num_joints_);
  joint_effort_.resize(num_joints_);
  joint_temperature_.resize(num_joints_);
  joint_temperature_variance_.resize(num_joints_);
  joint_position_command_.resize(num_joints_);
  joint_velocity_command_.resize(num_joints_);
  joint_effort_command_.resize(num_joints_);
  soft_limits_.resize(num_joints_);

  for (int i = 0; i < num_joints_; ++i)
  {
    SoftJointLimits soft_limits;
    getSoftJointLimits(model.getJoint(joint_names_[i]), soft_limits);
    ROS_DEBUG("[%s] Soft limits set to (%f, %f)", joint_names_[i].c_str(), soft_limits.min_position,
              soft_limits.max_position);
    soft_limits_[i] = soft_limits;
  }

  initiateIMC();

  // Create march_pdb_state interface
  MarchPdbStateHandle marchPdbStateHandle("PDBhandle", &power_distribution_board_read_,
                                          &master_shutdown_allowed_command, &enable_high_voltage_command,
                                          &power_net_on_off_command_);
  march_pdb_interface.registerHandle(marchPdbStateHandle);

  registerInterface(&march_temperature_interface);
  registerInterface(&march_pdb_interface);
  registerInterface(&joint_state_interface_);
  registerInterface(&position_joint_interface_);
  registerInterface(&effort_joint_interface_);
  registerInterface(&positionJointSoftLimitsInterface);
  registerInterface(&effortJointSoftLimitsInterface);

  hasPowerDistributionBoard = marchRobot.getPowerDistributionBoard()->getSlaveIndex() != -1;
  if (hasPowerDistributionBoard)
  {
    for (int i = 0; i < num_joints_; i++)
    {
      int netNumber = marchRobot.getJoint(joint_names_[i]).getNetNumber();
      if (netNumber == -1)
      {
        std::ostringstream errorStream;
        errorStream << "Joint " << joint_names_[i].c_str() << " has no net number";
        throw std::runtime_error(errorStream.str());
      }
      while (!marchRobot.getPowerDistributionBoard()->getHighVoltage().getNetOperational(netNumber))
      {
        marchRobot.getPowerDistributionBoard()->getHighVoltage().setNetOnOff(true, netNumber);
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
  for (int i = 0; i < num_joints_; ++i)
  {
    march::Joint joint = marchRobot.getJoint(joint_names_[i]);
    // Create joint state interface
    JointStateHandle jointStateHandle(joint.getName(), &joint_position_[i], &joint_velocity_[i], &joint_effort_[i]);
    joint_state_interface_.registerHandle(jointStateHandle);

    // Retrieve joint (soft) limits from the urdf
    JointLimits limits;
    getJointLimits(model.getJoint(joint.getName()), limits);

    if (joint.getActuationMode() == march::ActuationMode::position)
    {
      // Create position joint interface
      JointHandle jointPositionHandle(jointStateHandle, &joint_position_command_[i]);
      position_joint_interface_.registerHandle(jointPositionHandle);

      // Create joint limit interface
      PositionJointSoftLimitsHandle jointLimitsHandle(jointPositionHandle, limits, soft_limits_[i]);
      positionJointSoftLimitsInterface.registerHandle(jointLimitsHandle);
    }
    else if (joint.getActuationMode() == march::ActuationMode::torque)
    {
      // Create effort joint interface
      JointHandle jointEffortHandle(jointStateHandle, &joint_effort_command_[i]);
      effort_joint_interface_.registerHandle(jointEffortHandle);

      // Create joint effort limit interface
      EffortJointSoftLimitsHandle jointEffortLimitsHandle(jointEffortHandle, limits, soft_limits_[i]);
      effortJointSoftLimitsInterface.registerHandle(jointEffortLimitsHandle);
    }

    // Create velocity joint interface
    JointHandle jointVelocityHandle(jointStateHandle, &joint_velocity_command_[i]);
    velocity_joint_interface_.registerHandle(jointVelocityHandle);

    // Create march_state interface
    MarchTemperatureSensorHandle marchTemperatureSensorHandle(joint_names_[i], &joint_temperature_[i],
                                                              &joint_temperature_variance_[i]);
    march_temperature_interface.registerHandle(marchTemperatureSensorHandle);

    // Enable high voltage on the IMC
    if (joint.canActuate())
    {
      if (hasPowerDistributionBoard)
      {
        int net_number = joint.getNetNumber();
        if (net_number != -1)
        {
          marchRobot.getPowerDistributionBoard()->getHighVoltage().setNetOnOff(true, net_number);
        }
        else
        {
          ROS_FATAL("Joint %s has no high voltage net number", joint.getName().c_str());
          throw std::runtime_error("Joint has no high voltage net number");
        }
      }
      joint.prepareActuation();

      // Set the first target as the current position
      joint_position_[i] = joint.getAngleRad();
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
  controller_manager_.reset(new controller_manager::ControllerManager(this, nh_));
}

void MarchHardwareInterface::update(const ros::Duration& elapsed_time)
{
  read(elapsed_time);
  validate();
  controller_manager_->update(ros::Time::now(), elapsed_time);
  write(elapsed_time);
}

void MarchHardwareInterface::validate()
{
  for (int i = 0; i < num_joints_; i++)
  {
    this->outsideLimitsCheck(i);
    this->iMotionCubeStateCheck(i);
  }
}

void MarchHardwareInterface::read(const ros::Duration& elapsed_time)
{
  for (int i = 0; i < num_joints_; i++)
  {
    float oldPosition = joint_position_[i];

    march::Joint joint = marchRobot.getJoint(joint_names_[i]);

    joint_position_[i] = joint.getAngleRad();

    if (joint.hasTemperatureGES())
    {
      joint_temperature_[i] = joint.getTemperature();
    }

    // Get velocity from encoder position
    float joint_velocity = (joint_position_[i] - oldPosition) * 1 / elapsed_time.toSec();

    // Apply exponential smoothing to velocity obtained from encoder with
    // alpha=0.2
    joint_velocity_[i] = filters::exponentialSmoothing(joint_velocity, joint_velocity_[i], 0.2);

    joint_effort_[i] = joint.getTorque();

    ROS_DEBUG("Joint %s: read position %f", joint_names_[i].c_str(), joint_position_[i]);
  }

  this->updateIMotionCubeState();

  if (hasPowerDistributionBoard)
  {
    power_distribution_board_read_ = *marchRobot.getPowerDistributionBoard();

    if (!power_distribution_board_read_.getHighVoltage().getHighVoltageEnabled())
    {
      ROS_WARN_THROTTLE(10, "All-High-Voltage disabled");
    }
  }
}

void MarchHardwareInterface::write(const ros::Duration& elapsed_time)
{
  for (int i = 0; i < num_joints_; i++)
  {
    // Enlarge joint_effort_command because ROS control limits the pid values to a certain maximum
    joint_effort_command_[i] = joint_effort_command_[i] * 1000;
  }

  // Enforce limits on all joints in effort mode
  effortJointSoftLimitsInterface.enforceLimits(elapsed_time);

  // Enforce limits on all joints in position mode
  positionJointSoftLimitsInterface.enforceLimits(elapsed_time);

  for (int i = 0; i < num_joints_; i++)
  {
    march::Joint joint = marchRobot.getJoint(joint_names_[i]);

    if (joint.canActuate())
    {
      ROS_DEBUG("After limits: Trying to actuate joint %s, to %lf rad, %f "
                "speed, %f effort.",
                joint_names_[i].c_str(), joint_position_command_[i], joint_velocity_command_[i],
                joint_effort_command_[i]);

      if (joint.getActuationMode() == march::ActuationMode::position)
      {
        joint.actuateRad(static_cast<float>(joint_position_command_[i]));
      }
      else if (joint.getActuationMode() == march::ActuationMode::torque)
      {
        joint.actuateTorque(static_cast<int>(joint_effort_command_[i]));
      }
    }
  }

  this->updateAfterLimitJointCommand();

  if (hasPowerDistributionBoard)
  {
    updatePowerDistributionBoard();
  }
}

void MarchHardwareInterface::initiateIMC()
{
  ROS_INFO("Resetting all IMC on initialization");
  for (const std::string& joint_name : joint_names_)
  {
    Joint joint = marchRobot.getJoint(joint_name);
    joint.resetIMotionCube();
  }

  ROS_INFO("Restarting EtherCAT");
  marchRobot.stopEtherCAT();
  marchRobot.startEtherCAT();
}

void MarchHardwareInterface::updatePowerDistributionBoard()
{
  marchRobot.getPowerDistributionBoard()->setMasterOnline();
  marchRobot.getPowerDistributionBoard()->setMasterShutDownAllowed(master_shutdown_allowed_command);
  updateHighVoltageEnable();
  updatePowerNet();
}

void MarchHardwareInterface::updateHighVoltageEnable()
{
  try
  {
    if (marchRobot.getPowerDistributionBoard()->getHighVoltage().getHighVoltageEnabled() != enable_high_voltage_command)
    {
      marchRobot.getPowerDistributionBoard()->getHighVoltage().enableDisableHighVoltage(enable_high_voltage_command);
    }
    else if (!marchRobot.getPowerDistributionBoard()->getHighVoltage().getHighVoltageEnabled())
    {
      ROS_WARN_THROTTLE(2, "High voltage disabled");
    }
  }
  catch (std::exception& exception)
  {
    ROS_ERROR("%s", exception.what());
    ROS_DEBUG("Reverting the enable_high_voltage_command input, in attempt to prevent this exception is thrown "
              "again");
    enable_high_voltage_command = !enable_high_voltage_command;
  }
}

void MarchHardwareInterface::updatePowerNet()
{
  if (power_net_on_off_command_.getType() == PowerNetType::high_voltage)
  {
    try
    {
      if (marchRobot.getPowerDistributionBoard()->getHighVoltage().getNetOperational(
              power_net_on_off_command_.getNetNumber()) != power_net_on_off_command_.isOnOrOff())
      {
        marchRobot.getPowerDistributionBoard()->getHighVoltage().setNetOnOff(power_net_on_off_command_.isOnOrOff(),
                                                                             power_net_on_off_command_.getNetNumber());
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
      if (marchRobot.getPowerDistributionBoard()->getLowVoltage().getNetOperational(
              power_net_on_off_command_.getNetNumber()) != power_net_on_off_command_.isOnOrOff())
      {
        marchRobot.getPowerDistributionBoard()->getLowVoltage().setNetOnOff(power_net_on_off_command_.isOnOrOff(),
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

  // Clear msg of AfterLimitJointCommand
  after_limit_joint_command_pub_->msg_.name.clear();
  after_limit_joint_command_pub_->msg_.position_command.clear();
  after_limit_joint_command_pub_->msg_.effort_command.clear();

  for (int i = 0; i < num_joints_; i++)
  {
    march::Joint joint = marchRobot.getJoint(joint_names_[i]);

    after_limit_joint_command_pub_->msg_.header.stamp = ros::Time::now();
    after_limit_joint_command_pub_->msg_.name.push_back(joint.getName());
    after_limit_joint_command_pub_->msg_.position_command.push_back(joint_position_command_[i]);
    after_limit_joint_command_pub_->msg_.effort_command.push_back(joint_effort_command_[i]);
  }

  after_limit_joint_command_pub_->unlockAndPublish();
}

void MarchHardwareInterface::updateIMotionCubeState()
{
  if (!imc_state_pub_->trylock())
  {
    return;
  }
  // Clear msg of IMotionCubeStates
  imc_state_pub_->msg_.joint_names.clear();
  imc_state_pub_->msg_.status_word.clear();
  imc_state_pub_->msg_.detailed_error.clear();
  imc_state_pub_->msg_.motion_error.clear();
  imc_state_pub_->msg_.state.clear();
  imc_state_pub_->msg_.detailed_error_description.clear();
  imc_state_pub_->msg_.motion_error_description.clear();
  imc_state_pub_->msg_.motor_current.clear();
  imc_state_pub_->msg_.motor_voltage.clear();

  for (int i = 0; i < num_joints_; i++)
  {
    march::IMotionCubeState iMotionCubeState = marchRobot.getJoint(joint_names_[i]).getIMotionCubeState();
    imc_state_pub_->msg_.header.stamp = ros::Time::now();
    imc_state_pub_->msg_.joint_names.push_back(joint_names_[i]);
    imc_state_pub_->msg_.status_word.push_back(iMotionCubeState.statusWord);
    imc_state_pub_->msg_.detailed_error.push_back(iMotionCubeState.detailedError);
    imc_state_pub_->msg_.motion_error.push_back(iMotionCubeState.motionError);
    imc_state_pub_->msg_.state.push_back(iMotionCubeState.state.getString());
    imc_state_pub_->msg_.detailed_error_description.push_back(iMotionCubeState.detailedErrorDescription);
    imc_state_pub_->msg_.motion_error_description.push_back(iMotionCubeState.motionErrorDescription);
    imc_state_pub_->msg_.motor_current.push_back(iMotionCubeState.motorCurrent);
    imc_state_pub_->msg_.motor_voltage.push_back(iMotionCubeState.motorVoltage);
  }

  imc_state_pub_->unlockAndPublish();
}

void MarchHardwareInterface::iMotionCubeStateCheck(int joint_index)
{
  {
    march::IMotionCubeState iMotionCubeState = marchRobot.getJoint(joint_names_[joint_index]).getIMotionCubeState();
    if (iMotionCubeState.state == march::IMCState::fault)
    {
      std::ostringstream errorStream;
      errorStream << "IMotionCube of joint " << joint_names_[joint_index].c_str() << " is in fault state "
                  << iMotionCubeState.state.getString() << std::endl;
      errorStream << "Detailed Error: " << iMotionCubeState.detailedErrorDescription << "("
                  << iMotionCubeState.detailedError << ")" << std::endl;
      errorStream << "Motion Error: " << iMotionCubeState.motionErrorDescription << "(" << iMotionCubeState.motionError
                  << ")" << std::endl;

      ROS_FATAL("%s", errorStream.str().c_str());
      throw std::runtime_error(errorStream.str());
    }
  }
}

void MarchHardwareInterface::outsideLimitsCheck(int joint_index)
{
  march::Joint joint = marchRobot.getJoint(joint_names_[joint_index]);
  if (joint_position_[joint_index] < soft_limits_[joint_index].min_position ||
      joint_position_[joint_index] > soft_limits_[joint_index].max_position)
  {
    ROS_ERROR_THROTTLE(1, "Joint %s is outside of its soft limits (%f, %f). Actual position: %f",
                       joint_names_[joint_index].c_str(), soft_limits_[joint_index].min_position,
                       soft_limits_[joint_index].max_position, joint_position_[joint_index]);

    if (joint.canActuate())
    {
      std::ostringstream errorStream;
      errorStream << "Joint " << joint_names_[joint_index].c_str() << " is out of its soft limits ("
                  << soft_limits_[joint_index].min_position << ", " << soft_limits_[joint_index].max_position
                  << "). Actual position: " << joint_position_[joint_index];
      ROS_FATAL("%s", errorStream.str().c_str());
      throw ::std::runtime_error(errorStream.str());
    }
  }
}
