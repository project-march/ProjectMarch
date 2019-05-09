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
  march4cpp::MarchRobot march4 = march4cpp::MarchRobot(jointList, pdb, "enp2s0", ecatCycleTime);
  march4.startEtherCAT();

  if (!march4.isEthercatOperational())
  {
    ROS_FATAL("EtherCAT is not operational");
    return 0;
  }

  //     ros::init(argc, argv, "dummy");
  //     ros::NodeHandle nh;
  //     ros::Rate rate(10);

  //  if (march4.getJoint("test_joint").canActuate())
  //  {
  //    march4.getJoint("test_joint").prepareActuation();
  //  }

  ROS_INFO("march4 initialized");

  march4.getPowerDistributionBoard().setMasterOk(true);
  bool flip = true;
//  while (true)
//  {
    //      ROS_INFO("getPowerDistributionBoardCurrent: %f",
    //      march4.getPowerDistributionBoard().getPowerDistributionBoardCurrent());
    //      ROS_INFO("getLowVoltageNet1Current: %f", march4.getPowerDistributionBoard().getLowVoltageNet1Current());
    //      ROS_INFO("getLowVoltageNet2Current: %f", march4.getPowerDistributionBoard().getLowVoltageNet2Current());
    //      ROS_INFO("getHighVoltageNetCurrent: %f", march4.getPowerDistributionBoard().getHighVoltageNetCurrent());

    //      ROS_INFO("1 getLowVoltageNetOperational: %d",
    //      march4.getPowerDistributionBoard().getHighVoltageNetOperational(3));
//    flip = !flip;
//    if (flip)
//    {
//      ROS_INFO("turning on");
//    }
//    else
//    {
//      ROS_INFO("turning off");
//    }
//    march4.getPowerDistributionBoard().setHighVoltageNetOnOff(flip, 1);
//      march4.getPowerDistributionBoard().setMasterOk(true);
      ROS_INFO("MASTER OK?");
      sleep(5);
      march4.getPowerDistributionBoard().setHighVoltageNetOnOff(true, 1);
      ROS_INFO("1");
//      march4.stopEtherCAT();
//      sleep(10);
//      march4.getPowerDistributionBoard().setHighVoltageNetOnOff(true, 2);
//      ROS_INFO("2");
//      sleep(10);
//      march4.getPowerDistributionBoard().setHighVoltageNetOnOff(true, 3);
//      ROS_INFO("3");
//      sleep(10);
//      march4.getPowerDistributionBoard().setHighVoltageNetOnOff(true, 4);
//      ROS_INFO("4");
//      sleep(10);
//      march4.getPowerDistributionBoard().setHighVoltageNetOnOff(true, 5);
//      ROS_INFO("5");
//      sleep(10);
//      march4.getPowerDistributionBoard().setHighVoltageNetOnOff(true, 6);
//      ROS_INFO("6");
//      sleep(10);

      //      ROS_INFO("1 getLowVoltageNetOperational: %d",
    //      march4.getPowerDistributionBoard().getHighVoltageNetOperational(3));
    //        ROS_INFO("1 getLowVoltageNetCurrent: %f", march4.getPowerDistributionBoard().getLowVoltageNetCurrent(1));
    //    ROS_INFO("2 getLowVoltageNetOperational: %d",
    //    march4.getPowerDistributionBoard().getLowVoltageNetOperational(2));
    //    ROS_INFO("2 getPowerDistributionBoard: %f", march4.getPowerDistributionBoard().getLowVoltageNetCurrent(2));
//    sleep(30);

    //    for (int i = 1; i < 9; i++)
    //    {
    //      ROS_INFO("getHighVoltageOvercurrentTrigger index(%d) value: %d", i,
    //               march4.getPowerDistributionBoard().getHighVoltageOvercurrentTrigger(i));
    //    }
    //      ROS_INFO("getMasterShutdownRequested: %d",
    //      march4.getPowerDistributionBoard().getMasterShutdownRequested());
    //      if(march4.getPowerDistributionBoard().getMasterShutdownRequested()){
    //          march4.getPowerDistributionBoard().setMasterShutDownAllowed(true);
    //          march4.getPowerDistributionBoard().setMasterOk(false);
    //      }
//  }

//  ROS_INFO_STREAM("Angle: " << march4.getJoint("test_joint").getAngleRad());

//  march4.stopEtherCAT();
}
