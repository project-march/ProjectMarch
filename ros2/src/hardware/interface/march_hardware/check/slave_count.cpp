// Copyright 2019 Project March.

#include <ros/ros.h>

// SOEM ignores normal C header behaviour so these
// libraries HAVE to be included in this specific
// order.
// clang-format off
#include <soem/ethercattype.h>
#include <soem/nicdrv.h>
#include <soem/ethercatbase.h>
#include <soem/ethercatmain.h>
#include <soem/ethercatdc.h>
#include <soem/ethercatcoe.h>
#include <soem/ethercatfoe.h>
#include <soem/ethercatconfig.h>
#include <soem/ethercatprint.h>
// clang-format on

int main(int argc, char** argv)
{
    ros::init(argc, argv, "slave_count_check");
    ros::NodeHandle nh;
    const std::string param = "/march/check/slave_count";

    ROS_INFO("Trying to start EtherCAT");
    std::string if_name = argv[1];

    // Initialise SOEM, bind socket to if_name
    if (!ec_init(&if_name[0])) {
        ROS_FATAL("No socket connection on %s. Confirm that you have selected "
                  "the right if_name",
            if_name.c_str());
        nh.setParam(param, /*i=*/0);
        return 1;
    }
    ROS_INFO("ec_init on %s succeeded", if_name.c_str());

    // Find and auto-config slaves
    if (ec_config_init(FALSE) <= 0) {
        ROS_FATAL("No slaves found, shutting down. Confirm that you have "
                  "selected the right if_name.");
        ROS_FATAL("Check that the first slave is connected properly");
        nh.setParam(param, /*i=*/0);
        return 1;
    }
    ROS_INFO("%d slave(s) found and initialized.", ec_slavecount);

    nh.setParam(param, ec_slavecount);
    ros::shutdown();
    return 0;
}
