// Copyright 2018 Project March.

#include <algorithm>
#include <string>
#include <vector>

#include <ros/ros.h>

#include <march_hardware/Encoder.h>
#include <march_hardware/Joint.h>
#include <march_hardware/TemperatureSensor.h>

#include <march_hardware/EtherCAT/EthercatIO.h>

#include <march_hardware/MarchRobot.h>

namespace march4cpp
{
MarchRobot::MarchRobot(::std::vector<Joint> jointList, ::std::string ifName, int ecatCycleTime)
{
  this->jointList = std::move(jointList);
  this->powerDistributionBoard = std::unique_ptr<PowerDistributionBoard>(new PowerDistributionBoard());
  ethercatMaster.reset(new EthercatMaster(&this->jointList, ifName, this->getMaxSlaveIndex(), ecatCycleTime));
}

MarchRobot::MarchRobot(::std::vector<Joint> jointList, PowerDistributionBoard powerDistributionBoard,
                       ::std::string ifName, int ecatCycleTime)
{
  this->jointList = std::move(jointList);
  this->powerDistributionBoard =
      std::unique_ptr<PowerDistributionBoard>(new PowerDistributionBoard(powerDistributionBoard));
  ethercatMaster.reset(new EthercatMaster(&this->jointList, ifName, this->getMaxSlaveIndex(), ecatCycleTime));
}

void MarchRobot::startEtherCAT()
{
  if (!hasValidSlaves())
  {
    ROS_FATAL("Slaves are not configured properly. Confirm the slave indices "
              "are correct.");
    return;
  }

  ROS_INFO("Slave configuration is non-conflicting");

  if (ethercatMaster->isOperational)
  {
    ROS_ERROR("Trying to start EtherCAT while it is already active.");
    return;
  }
  ethercatMaster->start();
}

void MarchRobot::stopEtherCAT()
{
  if (!ethercatMaster->isOperational)
  {
    ROS_ERROR("Trying to stop EtherCAT while it is not active.");
    return;
  }

  ethercatMaster->stop();
}

int MarchRobot::getMaxSlaveIndex()
{
  int maxSlaveIndex = -1;

  for (int i = 0; i < jointList.size(); i++)
  {
    int temperatureSlaveIndex = jointList.at(i).getTemperatureGESSlaveIndex();
    if (temperatureSlaveIndex > maxSlaveIndex)
    {
      maxSlaveIndex = temperatureSlaveIndex;
    }

    int iMotionCubeSlaveIndex = jointList.at(i).getIMotionCubeSlaveIndex();

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

  for (int i = 0; i < jointList.size(); i++)
  {
    if (jointList[i].hasTemperatureGES())
    {
      int temperatureSlaveIndex = jointList[i].getTemperatureGESSlaveIndex();
      temperatureSlaveIndices.push_back(temperatureSlaveIndex);
    }

    if (jointList[i].hasIMotionCube())
    {
      int iMotionCubeSlaveIndex = jointList[i].getIMotionCubeSlaveIndex();
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
  return ethercatMaster->isOperational;
}

Joint MarchRobot::getJoint(::std::string jointName)
{
  if (!ethercatMaster->isOperational)
  {
    ROS_WARN("Trying to access joints while ethercat is not operational. This "
             "may lead to incorrect sensor data.");
  }
  for (int i = 0; i < jointList.size(); i++)
  {
    if (jointList.at(i).getName() == jointName)
    {
      return jointList.at(i);
    }
  }

  ROS_ERROR("Could not find joint with name %s", jointName.c_str());
  throw ::std::runtime_error("Could not find joint with name " + jointName);
}

const std::unique_ptr<PowerDistributionBoard>& MarchRobot::getPowerDistributionBoard() const
{
  return powerDistributionBoard;
}

MarchRobot::~MarchRobot()
{
  stopEtherCAT();
}

}  // namespace march4cpp
