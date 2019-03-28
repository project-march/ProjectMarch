//
// Created by projectmarch on 28-3-19.
//

#include <iostream>
#include <march_hardware_configuration_parser/HardwareBuilder.h>
#include <march_hardware_configuration_parser/HardwareConfigExceptions.h>


HardwareBuilder::HardwareBuilder() {

}

march4cpp::Encoder HardwareBuilder::createEncoder(YAML::Node config) {
    std::vector<std::string> required_keys = {"resolution", "minPositionIU", "maxPositionIU", "zeroPositionIU", "safetyMarginRad"};
    for(std::vector<std::string>::size_type i = 0; i != required_keys.size(); i++) {
        if(config["encoder"][required_keys.at(i)].Type() == YAML::NodeType::Undefined){
            throw MissingKeyException(required_keys.at(i), "encoder");
        }
    }

    int resolution = config["encoder"]["resolution"].as<int>();
    int minPositionIU = config["encoder"]["minPositionIU"].as<int>();
    int maxPositionIU = config["encoder"]["maxPositionIU"].as<int>();
    int zeroPositionIU = config["encoder"]["zeroPositionIU"].as<int>();
    float safetyMarginRad = config["encoder"]["safetyMarginRad"].as<float>();
    return march4cpp::Encoder(resolution, minPositionIU, maxPositionIU, zeroPositionIU, safetyMarginRad);
}
