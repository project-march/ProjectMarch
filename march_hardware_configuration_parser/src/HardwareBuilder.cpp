//
// Created by projectmarch on 28-3-19.
//

#include <iostream>
#include <march_hardware_configuration_parser/HardwareBuilder.h>
#include <march_hardware_configuration_parser/HardwareConfigExceptions.h>


HardwareBuilder::HardwareBuilder() {

}

march4cpp::Encoder HardwareBuilder::createEncoder(YAML::Node config) {
    if(config["encoder"]["resolution"].Type() == YAML::NodeType::Undefined){
        throw MissingKeyException("resolution", "encoder");
    }

    if(config["encoder"]["minPositionIU"].Type() == YAML::NodeType::Undefined){
        throw MissingKeyException("minPositionIU", "encoder");
    }

    if(config["encoder"]["maxPositionIU"].Type() == YAML::NodeType::Undefined){
        throw MissingKeyException("maxPositionIU", "encoder");
    }

    if(config["encoder"]["zeroPositionIU"].Type() == YAML::NodeType::Undefined){
        throw MissingKeyException("zeroPositionIU", "encoder");
    }

    if(config["encoder"]["safetyMarginRad"].Type() == YAML::NodeType::Undefined){
        throw MissingKeyException("safetyMarginRad", "encoder");
    }
    int resolution = config["encoder"]["resolution"].as<int>();
    int minPositionIU = config["encoder"]["minPositionIU"].as<int>();
    int maxPositionIU = config["encoder"]["maxPositionIU"].as<int>();
    int zeroPositionIU = config["encoder"]["zeroPositionIU"].as<int>();
    float safetyMarginRad = config["encoder"]["safetyMarginRad"].as<float>();
    return march4cpp::Encoder(resolution, minPositionIU, maxPositionIU, zeroPositionIU, safetyMarginRad);
}
