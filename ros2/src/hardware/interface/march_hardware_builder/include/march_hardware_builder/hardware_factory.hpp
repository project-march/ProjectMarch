//
// Created by march on 5-7-22.
//

#ifndef BUILD_BASE_HARDWARE_FACTORY_HPP
#define BUILD_BASE_HARDWARE_FACTORY_HPP

#include "march_logger_cpp/base_logger.hpp"

class HardwareFactory {
public:
    explicit HardwareFactory(const march_logger::BaseLogger& logger);

private:

    const march_logger::BaseLogger& logger_;

};

#endif //BUILD_BASE_HARDWARE_FACTORY_HPP
