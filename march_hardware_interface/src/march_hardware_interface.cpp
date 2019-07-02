// Copyright 2019 Project March.

#include <joint_limits_interface/joint_limits.h>
#include <joint_limits_interface/joint_limits_interface.h>
#include <joint_limits_interface/joint_limits_urdf.h>
#include <sstream>
#include <bitset>

#include <march_hardware/MarchRobot.h>

#include <march_hardware_interface/PowerNetOnOffCommand.h>
#include <march_hardware_interface/march_hardware_interface.h>

#include <urdf/model.h>

using joint_limits_interface::JointLimits;
using joint_limits_interface::SoftJointLimits;
using joint_limits_interface::PositionJointSoftLimitsHandle;
using joint_limits_interface::PositionJointSoftLimitsInterface;

namespace march_hardware_interface
{
MarchHardwareInterface::MarchHardwareInterface(ros::NodeHandle& nh, AllowedRobot robotName)
  : nh_(nh), marchRobot(HardwareBuilder(robotName).createMarchRobot())
{
  init();
  controller_manager_.reset(new controller_manager::ControllerManager(this, nh_));
  nh_.param("/march/hardware_interface/loop_hz", loop_hz_, 100.0);
  ros::Duration update_freq = ros::Duration(1.0 / loop_hz_);
  non_realtime_loop_ = nh_.createTimer(update_freq, &MarchHardwareInterface::update, this);
}

MarchHardwareInterface::~MarchHardwareInterface() = default;

void MarchHardwareInterface::init()
{
//   initialize realtime publisher
  RtPublisherPtr rt_pub(new realtime_tools::RealtimePublisher<march_shared_resources::ImcErrorState>(
            this->nh_, "/march/imc/", 4));
  realtime_pubs_ = rt_pub;
  ROS_INFO("initializing the realtime publisher");

  // Start ethercat cycle in the hardware
  this->marchRobot.startEtherCAT();

  ROS_INFO("etherCAT has been started");

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

  // Print all joint positions on startup in case initialization fails.
  this->read();
  for (int i = 0; i < num_joints_; ++i)
  {
    ROS_INFO("Joint %s: first read position: %f", joint_names_[i].c_str(), joint_position_[i]);
  }

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
      marchRobot.getPowerDistributionBoard()->getHighVoltage().setNetOnOff(false, netNumber);
    }
  }
  else
  {
    ROS_WARN("Running without Power Distribution Board");
  }

  // Initialize interfaces for each joint
  for (int i = 0; i < num_joints_; ++i)
  {
    march4cpp::Joint joint = marchRobot.getJoint(joint_names_[i]);
    // Create joint state interface
    JointStateHandle jointStateHandle(joint.getName(), &joint_position_[i], &joint_velocity_[i], &joint_effort_[i]);
    joint_state_interface_.registerHandle(jointStateHandle);

    // Create position joint interface
    JointHandle jointPositionHandle(jointStateHandle, &joint_position_command_[i]);

    // Retrieve joint (soft) limits from the urdf
    JointLimits limits;
    getJointLimits(model.getJoint(joint.getName()), limits);
    SoftJointLimits softLimits;
    getSoftJointLimits(model.getJoint(joint.getName()), softLimits);

    // Create joint limit interface
    PositionJointSoftLimitsHandle jointLimitsHandle(jointPositionHandle, limits, softLimits);
    positionJointSoftLimitsInterface.registerHandle(jointLimitsHandle);

    position_joint_interface_.registerHandle(jointPositionHandle);

    // Set the first target as the current position
    this->read();
    joint_velocity_[i] = 0;
    joint_effort_[i] = 0;
    joint_position_command_[i] = joint_position_[i];

    if (joint_position_[i] < softLimits.min_position || joint_position_[i] > softLimits.max_position)
    {
      ROS_FATAL("Joint %s is outside of its softLimits (%f, %f). Actual position: %f", joint_names_[i].c_str(),
                softLimits.min_position, softLimits.max_position, joint_position_[i]);

      if (joint.canActuate())
      {
        std::ostringstream errorStream;
        errorStream << "Joint " << joint_names_[i].c_str() << " is out of its softLimits (" << softLimits.min_position
                    << ", " << softLimits.max_position << "). Actual position: " << joint_position_[i];
        throw ::std::invalid_argument(errorStream.str());
      }
    }

    // Create velocity joint interface
    JointHandle jointVelocityHandle(jointStateHandle, &joint_velocity_command_[i]);
    velocity_joint_interface_.registerHandle(jointVelocityHandle);

    // Create effort joint interface
    JointHandle jointEffortHandle(jointStateHandle, &joint_effort_command_[i]);
    effort_joint_interface_.registerHandle(jointEffortHandle);

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
    }
  }
}

void MarchHardwareInterface::update(const ros::TimerEvent& e)
{
  elapsed_time_ = ros::Duration(e.current_real - e.last_real);
  read(elapsed_time_);
  controller_manager_->update(ros::Time::now(), elapsed_time_);
  write(elapsed_time_);
}

void MarchHardwareInterface::read(ros::Duration elapsed_time)
{
  for (int i = 0; i < num_joints_; i++)
  {
    float oldPosition = joint_position_[i];

    joint_position_[i] = marchRobot.getJoint(joint_names_[i]).getAngleRad();

    if (marchRobot.getJoint(joint_names_[i]).hasTemperatureGES())
    {
      joint_temperature_[i] = marchRobot.getJoint(joint_names_[i]).getTemperature();
    }

    // Get velocity from encoder position
    float joint_velocity = (joint_position_[i] - oldPosition) * 1 / elapsed_time.toSec();

    // Apply exponential smoothing to velocity obtained from encoder with
    // alpha=0.2
    joint_velocity_[i] = filters::exponentialSmoothing(joint_velocity, joint_velocity_[i], 0.2);

    ROS_DEBUG("Joint %s: read position %f", joint_names_[i].c_str(), joint_position_[i]);

    realtime_pubs_->msg_.joint_names.clear();
    realtime_pubs_->msg_.state.clear();
    realtime_pubs_->msg_.status_word.clear();
    realtime_pubs_->msg_.detailed_error.clear();
    realtime_pubs_->msg_.motion_error.clear();
    realtime_pubs_->msg_.motion_error_description.clear();

    std::vector<uint16> IMotionCubeErrorStates = marchRobot.getJoint(joint_names_[i]).getIMotionCubeErrorState();
    std::string IMotionCubeState = this->getIMotionCubeState(IMotionCubeErrorStates[0]);
    std::string motionErrorDescription = this->parseMotionError(IMotionCubeErrorStates[2]);

    std::bitset<16> statusWord = IMotionCubeErrorStates[0];
    std::bitset<16> detailedError = IMotionCubeErrorStates[1];
    std::bitset<16> motionError = IMotionCubeErrorStates[2];

    realtime_pubs_->msg_.joint_names.push_back(joint_names_[i]);
    realtime_pubs_->msg_.state.push_back(IMotionCubeState);
    realtime_pubs_->msg_.status_word.push_back(statusWord.to_string());
    realtime_pubs_->msg_.detailed_error.push_back(detailedError.to_string());
    realtime_pubs_->msg_.motion_error.push_back(motionError.to_string());
    realtime_pubs_->msg_.motion_error_description.push_back(motionErrorDescription);

  }
  if (realtime_pubs_->trylock())
  {
    realtime_pubs_->unlockAndPublish();
  }

  if (hasPowerDistributionBoard)
  {
    power_distribution_board_read_ = *marchRobot.getPowerDistributionBoard();

    if (!power_distribution_board_read_.getHighVoltage().getHighVoltageEnabled())
    {
      ROS_WARN_THROTTLE(10, "All-High-Voltage disabled");
    }
  }
}

void MarchHardwareInterface::write(ros::Duration elapsed_time)
{
  positionJointSoftLimitsInterface.enforceLimits(elapsed_time);

  for (int i = 0; i < num_joints_; i++)
  {
    if (marchRobot.getJoint(joint_names_[i]).canActuate())
    {
      ROS_DEBUG("After limits: Trying to actuate joint %s, to %lf rad, %f "
                "speed, %f effort.",
                joint_names_[i].c_str(), joint_position_command_[i], joint_velocity_command_[i],
                joint_effort_command_[i]);
      marchRobot.getJoint(joint_names_[i]).actuateRad(static_cast<float>(joint_position_command_[i]));
    }
  }

  if (hasPowerDistributionBoard)
  {
    updatePowerDistributionBoard();
  }
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

std::string MarchHardwareInterface::getIMotionCubeState(uint16 statusWord)
{
    uint16 fiveBitMask = 0b0000000001001111;
    uint16 sixBitMask = 0b0000000001101111;

    uint16 notReadyToSwitchOn = 0b0000000000000000;
    uint16 switchOnDisabled = 0b0000000001000000;
    uint16 readyToSwitchOn = 0b0000000000100001;
    uint16 switchedOn = 0b0000000000100011;
    uint16 operationEnabled = 0b0000000000100111;
    uint16 quickStopActive = 0b0000000000000111;
    uint16 faultReactionActive = 0b0000000000001111;
    uint16 fault = 0b0000000000001000;

    uint16 statusWordFiveBitMasked = (statusWord & fiveBitMask);
    uint16 statusWordSixBitMasked = (statusWord & sixBitMask);

    if (statusWordFiveBitMasked == notReadyToSwitchOn)
    {
        return "Not Ready To Switch On";
    }
    else if (statusWordFiveBitMasked == switchOnDisabled)
    {
        return "Switch On Disabled";
    }
    else if (statusWordSixBitMasked == readyToSwitchOn)
    {
        return "Ready to Switch On";
    }
    else if (statusWordSixBitMasked == switchedOn)
    {
        return "Switched On";
    }
    else if (statusWordSixBitMasked == operationEnabled)
    {
        return "Operation Enabled";
    }
    else if (statusWordSixBitMasked == quickStopActive)
    {
        return "Quick Stop Active";
    }
    else if (statusWordFiveBitMasked == faultReactionActive)
    {
        return "Fault Reaction Active";
    }
    else if (statusWordFiveBitMasked == fault)
    {
        return "Fault";
    }
    else return "Not in a recognized IMC state";
}

std::string MarchHardwareInterface::parseMotionError(uint16 motionError)
{
  std::string errorDescription = "";
  std::vector<std::string> bitDescriptions = {};
  bitDescriptions.push_back("\tEtherCAT communication error. Reset by a Reset "
                            "Fault command or by Clear Error in the "
                            "EtherCAT State Machine.");
  bitDescriptions.push_back("\tShort-circuit. Set when protection is "
                            "triggered. .");
  bitDescriptions.push_back("\tInvalid setup data. Set when the EEPROM stored "
                            "setup data is not valid or not present.");
  bitDescriptions.push_back("\tControl error (position/speed error too big). "
                            "Set when protection is triggered. Reset "
                            "by a Reset Fault command.");
  bitDescriptions.push_back("\tCommunication error. Set when protection is "
                            "triggered. .");
  bitDescriptions.push_back("\tMotor position wraps around. Set when "
                            "protection is triggered. Reset by a Reset Fault "
                            "command.");
  bitDescriptions.push_back("\tPositive limit switch active. Set when LSP "
                            "input is in active state. Reset when LSP "
                            "input is inactive state");
  bitDescriptions.push_back("\tNegative limit switch active. Set when LSN "
                            "input is in active state. Reset when LSN "
                            "input is inactive state");
  bitDescriptions.push_back("\tOver current. Set when protection is triggered. "
                            "");
  bitDescriptions.push_back("\tI2T protection. Set when protection is "
                            "triggered. ");
  bitDescriptions.push_back("\tOver temperature motor. Set when protection is "
                            "triggered. Reset by a Reset Fault "
                            "command. This protection may be activated if the "
                            "motor has a PTC or NTC temperature "
                            "contact.");
  bitDescriptions.push_back("\tOver temperature drive. Set when protection is "
                            "triggered. Reset by a Reset Fault "
                            "command.");
  bitDescriptions.push_back("\tOver-voltage. Set when protection is triggered. "
                            "");
  bitDescriptions.push_back("\tUnder-voltage. Set when protection is triggered.");
  bitDescriptions.push_back("\tCommand error. This bit is set in several situations. They can be "
                            "distinguished either by the associated "
                            "emergency code, or in conjunction with other bits:\n"
                            "\t\t0xFF03 - Specified homing method not available\n"
                            "\t\t0xFF04 - A wrong mode is set in object 6060h, modes_of_operation\n"
                            "\t\t0xFF05 - Specified digital I/O line not available\n"
                            "\tA function is called during the execution of another function (+ set "
                            "bit 7 of object 6041h, statusword).\n"
                            "\tUpdate of operation mode received during a transition. This bit acts "
                            "just as a warning.");
  bitDescriptions.push_back("Drive disabled: emergency button connector not shorted");

  for (int i = 0; i < 16; i++)
  {
    if (get_bit(motionError, i) == 1)
    {
      errorDescription = errorDescription + bitDescriptions.at(i);
    }
  }
  return errorDescription;
}

bool MarchHardwareInterface::get_bit(uint16 value, int index)
{
  return static_cast<bool>(value & (1 << index));
}

}  // namespace march_hardware_interface
