// Copyright 2019 Project March.

#include <ros/ros.h>

#include <soem/ethercattype.h>
#include <soem/nicdrv.h>
#include <soem/ethercatbase.h>
#include <soem/ethercatmain.h>
#include <soem/ethercatdc.h>
#include <soem/ethercatcoe.h>
#include <soem/ethercatfoe.h>
#include <soem/ethercatconfig.h>
#include <soem/ethercatprint.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "slave_count_check");
  ros::NodeHandle nh;

  ROS_INFO("Trying to start EtherCAT");
  std::string ifname = "enp2s0";

  // Initialise SOEM, bind socket to ifname
  if (!ec_init(&ifname[0]))
  {
    ROS_FATAL("No socket connection on %s. Confirm that you have selected the right ifname", ifname.c_str());
    nh.setParam("/check/slave_count", 0);
    return 1;
  }
  ROS_INFO("ec_init on %s succeeded", ifname.c_str());

  // Find and auto-config slaves
  if (ec_config_init(FALSE) <= 0)
  {
    ROS_FATAL("No slaves found, shutting down. Confirm that you have selected the right ifname.");
    ROS_FATAL("Check that the first slave is connected properly");
    nh.setParam("/check/slave_count", 0);
    return 1;
  }
  ROS_INFO("%d slave(s) found and initialized.", ec_slavecount);

  nh.setParam("/check/slave_count", ec_slavecount);
  return 0;
}
