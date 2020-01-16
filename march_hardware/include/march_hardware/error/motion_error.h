// Copyright 2020 Project March.

#ifndef MARCH_HARDWARE_MOTION_ERROR_H
#define MARCH_HARDWARE_MOTION_ERROR_H
#include <bitset>
#include <string>

namespace march
{
namespace error
{
const size_t MOTION_ERRORS_SIZE = 16;
extern const char* MOTION_ERRORS[MOTION_ERRORS_SIZE];

const size_t DETAILED_MOTION_ERRORS_SIZE = 9;
extern const char* DETAILED_MOTION_ERRORS[DETAILED_MOTION_ERRORS_SIZE];

std::string parseMotionError(uint16_t motion_error);

std::string parseDetailedError(uint16_t detailed_error);

}
}
#endif  // MARCH_HARDWARE_MOTION_ERROR_H
