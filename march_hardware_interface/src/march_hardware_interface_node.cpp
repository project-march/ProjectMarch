// Copyright 2019 Project March.
#include <string>
#include <march_hardware_interface/march_hardware_interface.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "march_hardware_interface");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(2);
  spinner.start();

  std::string selectedRobotName = argv[1];
  ROS_INFO("Selected robot: %s", selectedRobotName.c_str());
  AllowedRobot selectedRobot = AllowedRobot(selectedRobotName);

  march_hardware_interface::MarchHardwareInterface march(nh, selectedRobot);

  ros::waitForShutdown();
  return 0;
}
