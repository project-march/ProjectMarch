// Copyright 2020 Project March.

#ifndef MARCH_HARDWARE_MOTION_ERROR_H
#define MARCH_HARDWARE_MOTION_ERROR_H
#include <bitset>
#include <string>

namespace march
{
namespace error
{
enum class ErrorRegister
{
  IMOTIONCUBE_MOTION_ERROR,
  IMOTIONCUBE_DETAILED_MOTION_ERROR,
  IMOTIONCUBE_SECOND_DETAILED_MOTION_ERROR,
  ODRIVE_AXIS_ERROR,
  ODRIVE_MOTOR_ERROR,
  ODRIVE_ENCODER_ERROR,
  ODRIVE_ENCODER_MANAGER_ERROR,
  ODRIVE_CONTROLLER_ERROR,
};

const size_t IMOTIONCUBE_MOTION_ERRORS_SIZE = 16;
extern const char* IMOTIONCUBE_MOTION_ERRORS[IMOTIONCUBE_MOTION_ERRORS_SIZE];

const size_t IMOTIONCUBE_DETAILED_MOTION_ERRORS_SIZE = 9;
extern const char* IMOTIONCUBE_DETAILED_MOTION_ERRORS[IMOTIONCUBE_DETAILED_MOTION_ERRORS_SIZE];

const size_t IMOTIONCUBE_SECOND_DETAILED_MOTION_ERROR_SIZE = 7;
extern const char* IMOTIONCUBE_SECOND_DETAILED_MOTION_ERRORS[IMOTIONCUBE_SECOND_DETAILED_MOTION_ERROR_SIZE];

const size_t ODRIVE_AXIS_ERRORS_SIZE = 8;
extern const char* ODRIVE_AXIS_ERRORS[ODRIVE_AXIS_ERRORS_SIZE];

const size_t ODRIVE_MOTOR_ERRORS_SIZE = 26;
extern const char* ODRIVE_MOTOR_ERRORS[ODRIVE_MOTOR_ERRORS_SIZE];

const size_t ODRIVE_ENCODER_ERRORS_SIZE = 11;
extern const char* ODRIVE_ENCODER_ERRORS[ODRIVE_ENCODER_ERRORS_SIZE];

const size_t ODRIVE_ENCODER_MANAGER_ERRORS_SIZE = 1;
extern const char* ODRIVE_ENCODER_MANAGER_ERRORS[ODRIVE_ENCODER_MANAGER_ERRORS_SIZE];

const size_t ODRIVE_CONTROLLER_ERRORS_SIZE = 7;
extern const char* ODRIVE_CONTROLLER_ERRORS[ODRIVE_CONTROLLER_ERRORS_SIZE];

// Add an error type to the description
void addErrorToDescription(size_t index, ErrorRegister error_register, std::string &description);

// Parse an uint16_t error
std::string parseError(uint16_t error, ErrorRegister);

// Parse an uint32_t error
std::string parseError(uint32_t error, ErrorRegister);

}  // namespace error
}  // namespace march
#endif  // MARCH_HARDWARE_MOTION_ERROR_H
