// Copyright 2018 Project March.

#include <algorithm>
#include <string>
#include <vector>

#include <ros/ros.h>

#include <march_hardware/Joint.h>
#include <march_hardware/TemperatureSensor.h>

#include <march_hardware/EtherCAT/EthercatIO.h>

#include <march_hardware/MarchRobot.h>
#include <march_hardware/error/hardware_exception.h>

namespace march
{
MarchRobot::MarchRobot(::std::vector<Joint> jointList, urdf::Model urdf, ::std::string ifName, int ecatCycleTime)
  : jointList(std::move(jointList))
  , urdf_(std::move(urdf))
  , ethercatMaster(ifName, this->getMaxSlaveIndex(), ecatCycleTime)
{
}

MarchRobot::MarchRobot(::std::vector<Joint> jointList, urdf::Model urdf, PowerDistributionBoard powerDistributionBoard,
                       ::std::string ifName, int ecatCycleTime)
  : jointList(std::move(jointList))
  , urdf_(std::move(urdf))
  , ethercatMaster(ifName, this->getMaxSlaveIndex(), ecatCycleTime)
  , powerDistributionBoard(powerDistributionBoard)
{
}

void MarchRobot::startEtherCAT()
{
  if (!hasValidSlaves())
  {
    throw error::HardwareException(error::ErrorType::INVALID_SLAVE_CONFIGURATION);
  }

  ROS_INFO("Slave configuration is non-conflicting");

  if (ethercatMaster.isOperational())
  {
    ROS_WARN("Trying to start EtherCAT while it is already active.");
    return;
  }
  ethercatMaster.start(this->jointList);
}

void MarchRobot::stopEtherCAT()
{
  if (!ethercatMaster.isOperational())
  {
    ROS_WARN("Trying to stop EtherCAT while it is not active.");
    return;
  }

  ethercatMaster.stop();
}

int MarchRobot::getMaxSlaveIndex()
{
  int maxSlaveIndex = -1;

  for (Joint& joint : jointList)
  {
    int temperatureSlaveIndex = joint.getTemperatureGESSlaveIndex();
    if (temperatureSlaveIndex > maxSlaveIndex)
    {
      maxSlaveIndex = temperatureSlaveIndex;
    }

    int iMotionCubeSlaveIndex = joint.getIMotionCubeSlaveIndex();

    if (iMotionCubeSlaveIndex > maxSlaveIndex)
    {
      maxSlaveIndex = iMotionCubeSlaveIndex;
    }
  }
  return maxSlaveIndex;
}

bool MarchRobot::hasValidSlaves()
{
  ::std::vector<int> iMotionCubeIndices;
  ::std::vector<int> temperatureSlaveIndices;

  for (auto& joint : jointList)
  {
    if (joint.hasTemperatureGES())
    {
      int temperatureSlaveIndex = joint.getTemperatureGESSlaveIndex();
      temperatureSlaveIndices.push_back(temperatureSlaveIndex);
    }

    if (joint.hasIMotionCube())
    {
      int iMotionCubeSlaveIndex = joint.getIMotionCubeSlaveIndex();
      iMotionCubeIndices.push_back(iMotionCubeSlaveIndex);
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

  slaveIndices.reserve(iMotionCubeIndices.size() + temperatureSlaveIndices.size());
  slaveIndices.insert(slaveIndices.end(), iMotionCubeIndices.begin(), iMotionCubeIndices.end());
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
  return ethercatMaster.isOperational();
}

bool MarchRobot::getTrainReturned()
{
  return this->ethercatMaster.getTrainReturned();
}

void MarchRobot::setTrainReturned(bool train_returned)
{
  return this->ethercatMaster.setTrainReturned(train_returned);
}

int MarchRobot::getEthercatCycleTime() const
{
  return this->ethercatMaster.getCycleTime();
}

Joint MarchRobot::getJoint(::std::string jointName)
{
  if (!ethercatMaster.isOperational())
  {
    ROS_WARN("Trying to access joints while ethercat is not operational. This "
             "may lead to incorrect sensor data.");
  }
  for (auto& joint : jointList)
  {
    if (joint.getName() == jointName)
    {
      return joint;
    }
  }

  throw std::out_of_range("Could not find joint with name " + jointName);
}

PowerDistributionBoard& MarchRobot::getPowerDistributionBoard()
{
  return powerDistributionBoard;
}

const PowerDistributionBoard& MarchRobot::getPowerDistributionBoard() const
{
  return powerDistributionBoard;
}

MarchRobot::~MarchRobot()
{
  stopEtherCAT();
}

const urdf::Model& MarchRobot::getUrdf() const
{
  return this->urdf_;
}

}  // namespace march
