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

namespace march
{
MarchRobot::MarchRobot(::std::vector<Joint> jointList, urdf::Model urdf, ::std::string ifName, int ecatCycleTime)
  : jointList(std::move(jointList))
  , urdf_(std::move(urdf))
  , ethercatMaster(EthercatMaster(ifName, this->getMaxSlaveIndex(), ecatCycleTime))
{
}

MarchRobot::MarchRobot(::std::vector<Joint> jointList, urdf::Model urdf, PowerDistributionBoard powerDistributionBoard,
                       ::std::string ifName, int ecatCycleTime)
  : jointList(std::move(jointList))
  , urdf_(std::move(urdf))
  , ethercatMaster(EthercatMaster(ifName, this->getMaxSlaveIndex(), ecatCycleTime))
  , powerDistributionBoard(powerDistributionBoard)
{
}

void MarchRobot::startEtherCAT(bool do_reset_imc)
{
  if (!hasValidSlaves())
  {
    ROS_FATAL("Slaves are not configured properly. Confirm the slave indices "
              "are correct.");
    return;
  }

  ROS_INFO("Slave configuration is non-conflicting");

  if (ethercatMaster.isOperational())
  {
    ROS_ERROR("Trying to start EtherCAT while it is already active.");
    return;
  }
  ethercatMaster.start(this->jointList);

  if (do_reset_imc)
  {
    ROS_INFO("Resetting all IMC");
    resetIMC();

    ROS_INFO("Restarting the EtherCAT Master");
    ethercatMaster.stop();
    ethercatMaster.start(this->jointList);

  }
}

void MarchRobot::stopEtherCAT()
{
  if (!ethercatMaster.isOperational())
  {
    ROS_ERROR("Trying to stop EtherCAT while it is not active.");
    return;
  }

  ethercatMaster.stop();
}

void MarchRobot::resetIMC()
{
  ROS_INFO("Resetting all IMC on initialization");
  for (auto& joint : jointList)
  {
    joint.resetIMotionCube();
  }

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

  ROS_ERROR("Could not find joint with name %s", jointName.c_str());
  throw ::std::runtime_error("Could not find joint with name " + jointName);
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
  if (this->ethercatMaster.isOperational())
  {
    for (auto& joint : jointList)
    {
      joint.shutdown();
    }
  }

  stopEtherCAT();
}

const urdf::Model& MarchRobot::getUrdf() const
{
  return this->urdf_;
}

}  // namespace march
