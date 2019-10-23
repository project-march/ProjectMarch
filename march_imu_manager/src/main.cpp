// Copyright 2019 Project March.
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>

#include <xsensdeviceapi.h>
#include <xstypes.h>

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "march_imu_manager");
    ros::NodeHandle node;

    ROS_INFO("STARTED");
    ros::spin();
    return 0;
}
