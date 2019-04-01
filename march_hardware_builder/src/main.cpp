// Copyright 2019 Project March.

#include <iostream>
#include <yaml-cpp/yaml.h>
#include <march_hardware_builder/HardwareBuilder.h>

int main(int argc, char* argv[])
{
  const std::string filePath = "/home/projectmarch/Documents/march-iv/march_ws/src/hardware-interface/"
                               "march_hardware_builder/src/march3.yaml";
  //    YAML::Node config = YAML::LoadFile();

  HardwareBuilder hardwareBuilder = HardwareBuilder(filePath);
  hardwareBuilder.createMarchRobot();
  //    march4cpp::Joint joint = hardwareBuilder.createJoint(config, "test_joint");
  ROS_INFO("Done");
  //
  //
  //    const std::string username = config["username"].as<std::string>();
  //    const std::string password = config["password"].as<std::string>();
}
