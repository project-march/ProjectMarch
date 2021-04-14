// Copyright 2020 Project March.

#ifndef MARCH_HARDWARE_MOTION_ERROR_H
#define MARCH_HARDWARE_MOTION_ERROR_H
#include <bitset>
#include <string>

namespace march {
namespace error {
    enum class ErrorRegisters {
        MOTION_ERROR,
        DETAILED_ERROR,
        SECOND_DETAILED_ERROR
    };

    const size_t MOTION_ERRORS_SIZE = 16;
    extern const char* MOTION_ERRORS[MOTION_ERRORS_SIZE];

    const size_t DETAILED_MOTION_ERRORS_SIZE = 9;
    extern const char* DETAILED_MOTION_ERRORS[DETAILED_MOTION_ERRORS_SIZE];

    const size_t SECOND_DETAILED_MOTION_ERROR_SIZE = 7;
    extern const char*
        SECOND_DETAILED_MOTION_ERRORS[SECOND_DETAILED_MOTION_ERROR_SIZE];

    std::string parseError(uint16_t error, ErrorRegisters);

} // namespace error
} // namespace march
#endif // MARCH_HARDWARE_MOTION_ERROR_H
