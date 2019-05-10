// Copyright 2018 Project March.
#include <cmath>

#include <unistd.h>

#include <ros/ros.h>

#include <bitset>

#include <march_hardware/Joint.h>
#include <march_hardware/Encoder.h>
#include <march_hardware/TemperatureGES.h>
#include <march_hardware/MarchRobot.h>
#include "sensor_msgs/JointState.h"
#include <march_hardware/EtherCAT/EthercatIO.h>
#include <march_hardware/EtherCAT/EthercatSDO.h>
#include <march_hardware/PowerDistributionBoard.h>
#include <march_hardware/PDOmap.h>

int main(int argc, char** argv)
{
  march4cpp::TemperatureGES temperatureGES = march4cpp::TemperatureGES(1, 0);
  // TODO(ISHA, MARTIJN) double-check these numbers.
  // march4cpp::Encoder enc = march4cpp::Encoder(16, 37961, 59649, 39717, 0.05);
  // march4cpp::IMotionCube imc = march4cpp::IMotionCube(2, enc);
  march4cpp::Joint temp = march4cpp::Joint("test_joint", true, temperatureGES);

  std::vector<march4cpp::Joint> jointList;
  //  jointList.push_back(temp);

  int ecatCycleTime = 4;  // milliseconds

  NetMonitorOffsets currentOffsets = NetMonitorOffsets(5, 9, 13, 17, 3, 2, 1, 4);
  NetDriverOffsets netDriverOffsets = NetDriverOffsets(4, 3, 2);
  BootShutdownOffsets stateOffsets = BootShutdownOffsets(0, 0, 1);
  march4cpp::PowerDistributionBoard pdb =
      march4cpp::PowerDistributionBoard(1, currentOffsets, netDriverOffsets, stateOffsets);

  ROS_INFO_STREAM(pdb);
  march4cpp::MarchRobot march4 = march4cpp::MarchRobot(jointList, pdb, "enp2s0", ecatCycleTime);
  march4.startEtherCAT();

  if (!march4.isEthercatOperational())
  {
    ROS_FATAL("EtherCAT is not operational");
    return 0;
  }

  ROS_INFO("march4 initialized");

  //      ROS_INFO("1 getLowVoltageNetOperational: %d",

  march4.getPowerDistributionBoard()->setMasterOnline();


  // All gets:
  march4.getPowerDistributionBoard()->getHighVoltage().getNetOperational(1); // 1-8
  march4.getPowerDistributionBoard()->getHighVoltage().getOvercurrentTrigger(1); // 1-8
  march4.getPowerDistributionBoard()->getHighVoltage().getNetCurrent();
  march4.getPowerDistributionBoard()->getHighVoltage().getEmergencyButtonTrigger();

  march4.getPowerDistributionBoard()->getLowVoltage().getNetOperational(1); // 1-2
  march4.getPowerDistributionBoard()->getLowVoltage().getNetCurrent(1); // 1-2

  march4.getPowerDistributionBoard()->getPowerDistributionBoardCurrent();
  march4.getPowerDistributionBoard()->getMasterShutdownRequested();

  usleep(10000);

  march4.stopEtherCAT();
}
