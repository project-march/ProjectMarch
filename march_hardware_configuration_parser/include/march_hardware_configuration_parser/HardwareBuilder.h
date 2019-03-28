//
// Created by projectmarch on 28-3-19.
//

#ifndef MARCH_IV_HARDWAREBUILDER_H
#define MARCH_IV_HARDWAREBUILDER_H

#include <yaml-cpp/yaml.h>

#include <march_hardware/Encoder.h>

class HardwareBuilder {

public:

    HardwareBuilder();

    march4cpp::Encoder createEncoder(YAML::Node config);

};


#endif //MARCH_IV_HARDWAREBUILDER_H
