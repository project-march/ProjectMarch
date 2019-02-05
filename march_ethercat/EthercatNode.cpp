/*
 * EthercatNode       - Initialises and runs SOEM/EthercatMaster and the ros node
 *
 */

#include <stdio.h>
#include <ros/ros.h>
#include <std_msgs/UInt8.h>
#include "EthercatMaster.h"
#include "launch_parameters.h"

EthercatMaster* ethercatMasterPtr;

// Temporary method to set data sent to Template GES
// Called when published on /march/template/data
void SetTemplateData(std_msgs::UInt8 msg)
{
  // Todo: make this general purpose for any slave type
  ethercatMasterPtr->SetByte("TEMPLATEGES", 0, msg.data);
}

int main(int argc, char* argv[])
{
  // Initialize the ROS node and SOEM
  ros::init(argc, argv, "EthercatNode");
  // Create node handle
  ros::NodeHandle nh;
  // Initialize launch parameters
  // Todo: Get rid of launch parameters and replace with nodehandle methods
  LaunchParameters::init_parameters();
  ros::Rate rate(*LaunchParameters::get_ethercat_frequency());

  // Initialize publishers and subscribers and their callbacks
  if (*LaunchParameters::get_number_of_GES() > 0)
  {
    // TEMPLATE GES HANDLER
    new ros::Subscriber(nh.subscribe("march/template/data", 50, &SetTemplateData));

    // Test
    int nrofGES;
    nh.getParam("/NUMBER_OF_GES", nrofGES);
    printf("Number of GES: %d\n", nrofGES);
  }

  // Initialize EthercatMaster
  EthercatMaster ethercatMaster = EthercatMaster(nh);
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
