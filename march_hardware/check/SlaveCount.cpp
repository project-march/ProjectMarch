// Copyright 2019 Project March.

#include <ros/ros.h>

extern "C"
{
#include "ethercat.h"
}

int main(int argc, char** argv)

{
    ROS_INFO("Trying to start EtherCAT");
    std::string ifname = "enp2s0";

    // Initialise SOEM, bind socket to ifname
    if (!ec_init(ifname.c_str()))
    {
        ROS_FATAL("No socket connection on %s. Confirm that you have selected the right ifname", ifname.c_str());
        return 0;
    }
    ROS_INFO("ec_init on %s succeeded", ifname.c_str());

    // Find and auto-config slaves
    if (ec_config_init(FALSE) <= 0)
    {
        ROS_FATAL("No slaves found, shutting down. Confirm that you have selected the right ifname.");
        ROS_FATAL("Check that the first slave is connected properly");
        return 0;
    }
    ROS_INFO("%d slave(s) found and initialized.", ec_slavecount);
    return ec_slavecount;
}
