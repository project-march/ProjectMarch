// Copyright 2019 Project March.
#include <ros/ros.h>
#include <march_hardware_interface/march_hardware_interface.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "march_hardware_interface");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(2);

  if (argc < 2)
  {
    ROS_FATAL("Missing robot argument\nusage: march_hardware_interface_node ROBOT");
    return 1;
  }
  AllowedRobot selected_robot = AllowedRobot(argv[1]);
  ROS_INFO_STREAM("Selected robot: " << selected_robot);

  spinner.start();

  MarchHardwareInterface march(selected_robot);

  try
  {
    march.init(nh, nh);
  }
  catch (const std::exception& e)
  {
    ROS_FATAL("Hardware interface caught an exception during init: %s", e.what());
    return 1;
  }

  const double loop_hz = ros::param::param("~loop_hz", 100.0);
  ros::Rate rate(loop_hz);

  controller_manager::ControllerManager controller_manager(&march, nh);

  while (ros::ok())
  {
    ros::Time now = ros::Time::now();
    try
    {
      march.read(now, rate.expectedCycleTime());
      march.validate();
      controller_manager.update(now, rate.expectedCycleTime());
      march.write(now, rate.expectedCycleTime());
      rate.sleep();
    }
    catch (const std::exception& e)
    {
      ROS_FATAL("Hardware interface caught an exception during update: %s", e.what());
      return 1;
    }
  }

  return 0;
}
