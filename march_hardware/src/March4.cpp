// Copyright 2018 Project March.
#include <vector>

#include <ros/ros.h>

#include <march_hardware/Joint.h>
#include <march_hardware/TemperatureSensor.h>
#include <march_hardware/Encoder.h>

#include <march_hardware/EtherCAT/EthercatIO.h>

#include <march_hardware/March4.h>

namespace march4cpp
{
MARCH4::MARCH4()
{
//                        0    1    2    3    4    5    6    7    8    9    10   11   12   13
//      LIST_OF_SLAVES: [MAS, PDB, BPG, LHJ, LUG, LKJ, LLG, LAJ, RHJ, RUG, RKJ, RLG, RAJ, IPD]
  // TODO(Isha, Martijn) double-check these numbers.

    Encoder RHJenc = Encoder(16, 22134, 43436, 24515, 0.05); // Calibrated
    Encoder LHJenc = Encoder(16, 9746, 31557, 11830, 0.05); // Calibrated

    Encoder RKJenc = Encoder(16, 18120, 39941, 19000, 0.05); // Calibrated
    Encoder LKJenc = Encoder(16, 21924, 43734, 22552, 0.05);

    Encoder RAJenc = Encoder(12, 1086, 1490, 1301, 0.005);
    Encoder LAJenc = Encoder(12, 631, 1022, 918, 0.005);

    IMotionCube LHJimc = IMotionCube(3, LHJenc);
    IMotionCube LKJimc = IMotionCube(5, LKJenc);
  IMotionCube LAJimc = IMotionCube(7, LAJenc);
  IMotionCube RHJimc = IMotionCube(8, RHJenc);
  IMotionCube RKJimc = IMotionCube(10, RKJenc);
  IMotionCube RAJimc = IMotionCube(12, RAJenc);

  Joint leftHip = Joint("left_hip_joint", LHJimc);
  Joint leftKnee = Joint("left_knee_joint", LKJimc);
  Joint leftAnkle = Joint("left_ankle_joint", LAJimc);
  Joint rightHip = Joint("right_hip_joint", RHJimc);
  Joint rightKnee = Joint("right_knee_joint", RKJimc);
  Joint rightAnkle = Joint("right_ankle_joint", RAJimc);

  // TODO(Tim) should not create copy of joint for vector

  jointList.push_back(leftHip);
  jointList.push_back(leftKnee);
  jointList.push_back(leftAnkle);
  jointList.push_back(rightHip);
  jointList.push_back(rightKnee);
  jointList.push_back(rightAnkle);

  int ecatCycleTime = 4;  // milliseconds
  ethercatMaster.reset(new EthercatMaster(jointList, "enp3s0", this->getMaxSlaveIndex(), ecatCycleTime));
}

void MARCH4::startEtherCAT()
{
  if (!hasValidSlaves())
  {
    ROS_FATAL("Slaves are not configured properly. Confirm the slave indices are correct.");
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

void MARCH4::stopEtherCAT()
{

  if (!ethercatMaster->isOperational)
  {
    ROS_ERROR("Trying to stop EtherCAT while it is not active.");
    return;
  }
  ethercatMaster->stop();
}

int MARCH4::getMaxSlaveIndex()
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

bool MARCH4::hasValidSlaves()
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
  // Remove duplicate temperatureSlaveIndices so they don't trigger as duplicates later.
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

bool MARCH4::isEthercatOperational()
{
  return ethercatMaster->isOperational;
}

Joint MARCH4::getJoint(::std::string jointName)
{
  if (!ethercatMaster->isOperational)
  {
    ROS_WARN("Trying to access joints while ethercat is not operational. This may lead to incorrect sensor data.");
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

}  // namespace march4cpp
