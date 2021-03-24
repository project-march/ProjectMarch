// Copyright 2018 Project March.
#include "march_hardware/joint.h"
#include "march_hardware/march_robot.h"
#include "march_hardware/temperature/temperature_sensor.h"
#include "march_hardware/error/hardware_exception.h"

#include <algorithm>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <ros/ros.h>

namespace march
{
MarchRobot::MarchRobot(::std::vector<Joint> jointList, urdf::Model urdf, std::unique_ptr<EthercatMaster> ethercatMaster)
  : jointList(std::move(jointList)), urdf_(std::move(urdf)), ethercatMaster(std::move(ethercatMaster)), pdb_(nullptr)
{
}

MarchRobot::MarchRobot(::std::vector<Joint> jointList, urdf::Model urdf,
                       std::unique_ptr<PowerDistributionBoard> powerDistributionBoard,
                       std::unique_ptr<EthercatMaster> ethercatMaster)
  : jointList(std::move(jointList))
  , urdf_(std::move(urdf))
  , ethercatMaster(std::move(ethercatMaster))
  , pdb_(std::move(powerDistributionBoard))
{
}

MarchRobot::MarchRobot(::std::vector<Joint> jointList, urdf::Model urdf,
                       std::unique_ptr<PowerDistributionBoard> powerDistributionBoard)
  : jointList(std::move(jointList))
  , urdf_(std::move(urdf))
  , ethercatMaster(nullptr)
  , pdb_(std::move(powerDistributionBoard))
{
}

MarchRobot::MarchRobot(::std::vector<Joint> jointList, urdf::Model urdf,
                       std::unique_ptr<PowerDistributionBoard> powerDistributionBoard,
                       std::vector<PressureSole> pressureSoles)
  : jointList(std::move(jointList))
  , urdf_(std::move(urdf))
  , ethercatMaster(nullptr)
  , pdb_(std::move(powerDistributionBoard))
  , pressureSoles(std::move(pressureSoles))
{
}

void MarchRobot::startCommunication(bool reset_motor_controllers)
{
  if (!hasValidSlaves())
  {
    throw error::HardwareException(error::ErrorType::INVALID_SLAVE_CONFIGURATION);
  }

  ROS_INFO("Slave configuration is non-conflicting");

  if (this->isCommunicationOperational())
  {
    ROS_WARN("Trying to start Communication while it is already active.");
    return;
  }

  if (ethercatMaster != nullptr)
  {
    bool sw_reset = ethercatMaster->start(this->jointList);

    if (reset_motor_controllers || sw_reset)
    {
      ROS_DEBUG("Resetting all MotorControllers due to either: reset arg: %d or downloading of .sw fie: %d",
                reset_motor_controllers, sw_reset);
      resetMotorControllers();

      ROS_INFO("Restarting the EtherCAT Master");
      ethercatMaster->stop();
      sw_reset = ethercatMaster->start(this->jointList);
    }
  }
}

void MarchRobot::stopCommunication()
{
  if (!this->isCommunicationOperational())
  {
    ROS_WARN("Trying to stop Communication while it is not active.");
    return;
  }

  if (ethercatMaster != nullptr)
  {
    ethercatMaster->stop();
  }
}

void MarchRobot::resetMotorControllers()
{
  for (auto& joint : jointList)
  {
    joint.resetMotorController();
  }
}

bool MarchRobot::hasValidSlaves()
{
  ::std::vector<int> motorControllerIndices;
  ::std::vector<int> temperatureSlaveIndices;

  for (auto& joint : jointList)
  {
    if (joint.hasTemperatureGES())
    {
      int temperatureSlaveIndex = joint.getTemperatureGESSlaveIndex();
      temperatureSlaveIndices.push_back(temperatureSlaveIndex);
    }

    if (joint.getMotorControllerSlaveIndex() > -1)
    {
      int motorControllerSlaveIndex = joint.getMotorControllerSlaveIndex();
      motorControllerIndices.push_back(motorControllerSlaveIndex);
    }
  }
  // Multiple temperature sensors may be connected to the same slave.
  // Remove duplicate temperatureSlaveIndices so they don't trigger as
  // duplicates later.
  sort(temperatureSlaveIndices.begin(), temperatureSlaveIndices.end());
  temperatureSlaveIndices.erase(unique(temperatureSlaveIndices.begin(), temperatureSlaveIndices.end()),
                                temperatureSlaveIndices.end());

  // Merge the slave indices
  ::std::vector<int> slaveIndices;

  slaveIndices.reserve(motorControllerIndices.size() + temperatureSlaveIndices.size());
  slaveIndices.insert(slaveIndices.end(), motorControllerIndices.begin(), motorControllerIndices.end());
  slaveIndices.insert(slaveIndices.end(), temperatureSlaveIndices.begin(), temperatureSlaveIndices.end());

  if (slaveIndices.size() == 1)
  {
    ROS_INFO("Found configuration for 1 slave.");
    return true;
  }

  ROS_INFO("Found configuration for %lu slaves.", slaveIndices.size());

  // Sort the indices and check for duplicates.
  // If there are no duplicates, the configuration is valid.
  ::std::sort(slaveIndices.begin(), slaveIndices.end());
  auto it = ::std::unique(slaveIndices.begin(), slaveIndices.end());
  bool isUnique = (it == slaveIndices.end());
  return isUnique;
}

bool MarchRobot::isEthercatOperational()
{
  return ethercatMaster != nullptr && ethercatMaster->isOperational();
}

bool MarchRobot::isCommunicationOperational()
{
  return this->isEthercatOperational();
}

std::exception_ptr MarchRobot::getLastCommunicationException() const noexcept
{
  if (ethercatMaster != nullptr)
  {
    return this->ethercatMaster->getLastException();
  }
  return nullptr;
}

void MarchRobot::waitForUpdate()
{
  if (ethercatMaster != nullptr)
  {
    this->ethercatMaster->waitForPdo();
  }
}

int MarchRobot::getCycleTime() const
{
  if (ethercatMaster != nullptr)
  {
    return this->ethercatMaster->getCycleTime();
  }
  return 0;
}

Joint& MarchRobot::getJoint(::std::string jointName)
{
  for (auto& joint : jointList)
  {
    if (joint.getName() == jointName)
    {
      if (joint.getMotorControllerSlaveIndex() != -1)
      {
        if (!this->isCommunicationOperational())
        {
          ROS_WARN("Trying to access joints while ethercat is not operational. This "
                   "may lead to incorrect sensor data.");
        }
      }
      return joint;
    }
  }

  throw std::out_of_range("Could not find joint with name " + jointName);
}

Joint& MarchRobot::getJoint(size_t index)
{
  Joint& joint = this->jointList.at(index);

  if (joint.getMotorControllerSlaveIndex() != -1)
  {
    if (!this->isCommunicationOperational())
    {
      ROS_WARN("Trying to access joints while ethercat is not operational. This "
               "may lead to incorrect sensor data.");
    }
  }

  return joint;
}

size_t MarchRobot::size() const
{
  return this->jointList.size();
}

MarchRobot::iterator MarchRobot::begin()
{
  auto joint = this->jointList.begin();

  if (joint[0].getMotorControllerSlaveIndex() != -1)
  {
    if (!this->isCommunicationOperational())
    {
      ROS_WARN("Trying to access joints while ethercat is not operational. This "
               "may lead to incorrect sensor data.");
    }
  }

  return joint;
}

MarchRobot::iterator MarchRobot::end()
{
  return this->jointList.end();
}

bool MarchRobot::hasPowerDistributionboard() const
{
  return this->pdb_ != nullptr;
}

PowerDistributionBoard* MarchRobot::getPowerDistributionBoard() const
{
  return this->pdb_.get();
}

bool MarchRobot::hasPressureSoles() const
{
  return pressureSoles.size() > 0;
}

std::vector<PressureSole> MarchRobot::getPressureSoles() const
{
  return pressureSoles;
}

MarchRobot::~MarchRobot()
{
  stopCommunication();
}

const urdf::Model& MarchRobot::getUrdf() const
{
  return this->urdf_;
}

}  // namespace march
