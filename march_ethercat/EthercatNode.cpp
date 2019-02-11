/*
 * EthercatNode       - Initialises and runs SOEM/EthercatMaster and the ros node
 *
 */

#include <stdio.h>
#include <ros/ros.h>
#include <std_msgs/UInt8.h>
#include "EthercatMaster.h"
#include "TemplateGES.h"
#include "IMC.h"
#include "PDB.h"

// EthercatMaster* ethercatMasterPtr;
std::vector<Slave*> slaveList;

// Temporary method to set data sent to Template GES
// Called when published on /march/template/data
void SetTemplateDataCB(std_msgs::UInt8 msg)
{
  for (int i = 0; i < slaveList.size(); i++)
  {
    std::string slaveType = slaveList[i]->getType();
    if (slaveType == "TEMPLATEGES")
    {
      TemplateGES* tmpTemplateGES = (TemplateGES*)slaveList[i];
      tmpTemplateGES->setLedCommand(msg.data);
      printf("Set LED command to GES with name %s to value %i\n", tmpTemplateGES->getName().c_str(), msg.data);
    }
  }
}

// Creates slave objects from dictionary in launch file
std::vector<Slave*> initSlaves(ros::NodeHandle nh)
{
  std::vector<Slave*> slaves;
  std::map<std::string, std::string> slaveMap;
  nh.getParam(ros::this_node::getName() + "/slaves", slaveMap);
  std::map<std::string, std::string>::iterator i;
  for (i = slaveMap.begin(); i != slaveMap.end(); i++)
  {
    uint16 slaveNumber = (uint16)std::distance(slaveMap.begin(), i);
    if (i->second == "TEMPLATEGES")
    {
      slaves.push_back(new TemplateGES(i->first, slaveNumber));
    }
    else if (i->second == "IMC")
    {
      slaves.push_back(new IMC(i->first, slaveNumber));
    }
    else if (i->second == "PDB")
    {
      slaves.push_back(new PDB(i->first, slaveNumber));
    }
    else
    {
      slaves.push_back(new Slave(i->first, slaveNumber));
    }
  }
  return slaves;
}

// Initialize subscribers and publishers based on if certain slave types are present
void initTopics(ros::NodeHandle nh)
{
  int nrofTEMPLATEGES, nrofPDB, nrofIMC;
  nrofTEMPLATEGES = nrofPDB = nrofIMC = 0;
  for (int i = 0; i < slaveList.size(); i++)
  {
    std::string slaveType = slaveList[i]->getType();
    if (slaveType == "IMC")
    {
      nrofIMC++;
    }
    if (slaveType == "PDB")
    {
      nrofPDB++;
    }
    if (slaveType == "TEMPLATEGES")
    {
      nrofTEMPLATEGES++;
    }
  }
  if (nrofTEMPLATEGES > 0)
  {
    new ros::Subscriber(nh.subscribe("march/template/data", 50, &SetTemplateDataCB));
  }
}

// Find a slave by its name
Slave* getSlaveByName(std::vector<Slave*>* slaveList, std::string name)
{
  for (int i = 0; i < slaveList->size(); i++)
  {
    if (slaveList->at(i)->getName() == name)
    {
      return slaveList->at(i);
    }
  }
  printf("No slave found with name %s\n", name.c_str());
  return slaveList->at(0);
}
//--------------------------------------------------------------------
// Main
//--------------------------------------------------------------------
int main(int argc, char* argv[])
{
  ros::init(argc, argv, "EthercatNode");
  ros::NodeHandle nh;

  int EthercatCycleTime, EthercatFrequency;
  nh.getParam("/ETHERCAT_CYCLE_TIME", EthercatCycleTime);
  EthercatFrequency = 1000 / EthercatCycleTime;
  ros::Rate rate(EthercatFrequency);

  slaveList = initSlaves(nh);

  // Print all slave types
  for (int i = 0; i < slaveList.size(); i++)
  {
    printf("%s\n", slaveList.at(i)->type.c_str());
  }

  initTopics(nh);

  //  Initialize EthercatMaster
  EthercatMaster ethercatMaster = EthercatMaster(nh, slaveList);
  //  ethercatMasterPtr = &ethercatMaster;

  while (ros::ok() && ethercatMaster.inOP)
  {
    ethercatMaster.SendProcessData();
    ethercatMaster.ReceiveProcessData();
    ethercatMaster.PublishProcessData();
    ethercatMaster.MonitorSlaveConnection();
    ros::spinOnce();
    rate.sleep();
  }

//  for (int i = 0; i < slaveList.size(); i++)
//  {
//    delete slaveList.at(i);
//  }
  printf("End of EthercatNode\n");
  return (0);
}
