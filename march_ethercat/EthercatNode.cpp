/*
 * EthercatNode       - Initialises and runs SOEM/EthercatMaster and the ros node
 *
 */

#include <stdio.h>
#include <ros/ros.h>
#include <std_msgs/UInt8.h>
#include "EthercatMaster.h"
#include "launch_parameters.h"
#include "GES.h"

EthercatMaster* ethercatMasterPtr;

// Temporary method to set data sent to Template GES
// Called when published on /march/template/data
void SetTemplateDataCB(std_msgs::UInt8 msg)
{
  // Todo: make this general purpose for any slave type
  ethercatMasterPtr->SetByte("TEMPLATEGES", 0, msg.data);
}

void initSubscribers(ros::NodeHandle nh)
{
    // Todo: make this work with slave classes
    int nrofGES, nrofPDB, nrofIMC;
    nh.getParam("/NUMBER_OF_GES", nrofGES);
    nh.getParam("/NUMBER_OF_PDB", nrofPDB);
    nh.getParam("/NUMBER_OF_JOINTS", nrofIMC);
    if (nrofGES > 0)
    {
        // TEMPLATE GES
        new ros::Subscriber(nh.subscribe("march/template/data", 50, &SetTemplateDataCB));
    }
}

void initPublishers(ros::NodeHandle nh)
{

}

std::vector<Slave> initSlaves(ros::NodeHandle nh)
{
    std::vector<Slave> slaves;
    //Todo: Get amount of slaves, types and order from launch file

    slaves.push_back(GES("TEMPLATEGES", 1));
    return slaves;
}


int main(int argc, char* argv[])
{
  ros::init(argc, argv, "EthercatNode");
  ros::NodeHandle nh;

  // Todo: Get rid of launch parameters and replace with nodehandle methods
  LaunchParameters::init_parameters();

  int EthercatCycleTime, EthercatFrequency;
  nh.getParam("/ETHERCAT_CYCLE_TIME",EthercatCycleTime);
  EthercatFrequency = 1000/EthercatCycleTime;
  ros::Rate rate(EthercatFrequency);

  std::vector<Slave> slaveList = initSlaves(nh);

  initSubscribers(nh);
  initPublishers(nh);

  // Initialize EthercatMaster
  EthercatMaster ethercatMaster = EthercatMaster(nh, slaveList);
  ethercatMasterPtr = &ethercatMaster;

  while (ros::ok() && ethercatMaster.inOP)
  {
    ethercatMaster.SendProcessData();
    ethercatMaster.ReceiveProcessData();
    ethercatMaster.PublishProcessData();
    ethercatMaster.MonitorSlaveConnection();
    ros::spinOnce();
    rate.sleep();
  }

  printf("End of EthercatNode\n");
  return (0);
}
