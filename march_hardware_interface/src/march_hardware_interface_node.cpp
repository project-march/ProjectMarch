// Copyright 2019 Project March.
#include "march_hardware_interface/march_hardware_interface.h"

#include <controller_manager/controller_manager.h>
#include <ros/ros.h>

#include <march_hardware/error/hardware_exception.h>
#include <march_hardware_builder/hardware_builder.h>

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

  bool do_reset_imc = argv[2];
  ROS_INFO_STREAM("Resetting the IMC: " << do_reset_imc);

  spinner.start();

  HardwareBuilder builder(selected_robot);
  MarchHardwareInterface march(builder.createMarchRobot(), do_reset_imc);

  try
  {
    bool success = march.init(nh, nh);
    if (!success)
    {
      return 1;
    }
  }
  catch (const std::exception& e)
  {
    ROS_FATAL("Hardware interface caught an exception during init");
    ROS_FATAL("%s", e.what());
    return 1;
  }

  ros::Rate rate(
      ros::Duration(march.getEthercatCycleTime() * ros::param::param<int>("~control_loop_multiplier", 1) / 1000.0));

  controller_manager::ControllerManager controller_manager(&march, nh);

  while (ros::ok())
  {
    const ros::Time now = ros::Time::now();
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
      ROS_FATAL("Hardware interface caught an exception during update");
      ROS_FATAL("%s", e.what());
      return 1;
    }
  }

  return 0;
}
