#include <iostream>
#include <yaml-cpp/yaml.h>
#include <march_hardware_builder/HardwareBuilder.h>

int main (int argc, char* argv[])
{
    YAML::Node config = YAML::LoadFile("/home/ishadijcks/march-iv/march_ws/src/hardware-interface/march_hardware_builder/yaml/dummy_joint.yaml");


    HardwareBuilder hardwareBuilder = HardwareBuilder();
    march4cpp::Joint joint = hardwareBuilder.createJoint(config, "test_joint");
    ROS_INFO("Done");
//
//
//    const std::string username = config["username"].as<std::string>();
//    const std::string password = config["password"].as<std::string>();
}
