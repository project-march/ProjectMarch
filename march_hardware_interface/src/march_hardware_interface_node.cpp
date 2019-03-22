// Copyright 2019 Project March.

#include <march_hardware_interface/march_hardware_interface.h>
int main(int argc, char** argv)
{
  ros::init(argc, argv, "march_hardware_interface");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(1);
  spinner.start();
  march_hardware_interface::MarchHardwareInterface march(nh);
  ros::waitForShutdown()
  return 0;
}
