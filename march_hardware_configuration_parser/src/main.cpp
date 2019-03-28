#include <iostream>
#include <yaml-cpp/yaml.h>
#include <march_hardware_configuration_parser/HardwareBuilder.h>

int main (int argc, char* argv[])
{
    YAML::Node config = YAML::LoadFile("/home/projectmarch/Documents/march-iv/march_ws/src/hardware-interface/march_hardware_configuration_parser/yaml/encoder.yaml");


    HardwareBuilder hardwareBuilder = HardwareBuilder();
    march4cpp::Encoder encoder = hardwareBuilder.createEncoder(config);


    for (std::size_t i=0;i<config["joints"].size();i++) {
        auto joint_config = config["joints"][i].as<std::map<std::string, int>>();
        auto test = joint_config.begin()->first;
    }

//
//
//    const std::string username = config["username"].as<std::string>();
//    const std::string password = config["password"].as<std::string>();
}
