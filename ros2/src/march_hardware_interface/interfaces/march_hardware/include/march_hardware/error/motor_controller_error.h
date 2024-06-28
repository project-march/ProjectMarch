// Copyright 2020 Project March.

#ifndef MARCH_HARDWARE_MOTOR_CONTROLLER_ERROR_H
#define MARCH_HARDWARE_MOTOR_CONTROLLER_ERROR_H
#include <array>
#include <bitset>
#include <climits>
#include <string>

namespace march {
namespace error {
    enum class ErrorRegister {
        ODRIVE_ERROR,
        ODRIVE_AXIS_ERROR,
        ODRIVE_MOTOR_ERROR,
        ODRIVE_ENCODER_ERROR,
        ODRIVE_TORQUESENSOR_ERROR,
        ODRIVE_CONTROLLER_ERROR,
    };

    const size_t ODRIVE_ERRORS_SIZE = 8;
    extern const std::array<std::string, ODRIVE_ERRORS_SIZE> ODRIVE_ERRORS;

    const size_t ODRIVE_AXIS_ERRORS_SIZE = 21;
    extern const std::array<std::string, ODRIVE_AXIS_ERRORS_SIZE> ODRIVE_AXIS_ERRORS;

    const size_t ODRIVE_MOTOR_ERRORS_SIZE = 37;
    extern const std::array<std::string, ODRIVE_MOTOR_ERRORS_SIZE> ODRIVE_MOTOR_ERRORS;

    const size_t ODRIVE_ENCODER_ERRORS_SIZE = 13;
    extern const std::array<std::string, ODRIVE_ENCODER_ERRORS_SIZE> ODRIVE_ENCODER_ERRORS;

    const size_t ODRIVE_TORQUESENSOR_ERRORS_SIZE = 3;
    extern const std::array<std::string, ODRIVE_TORQUESENSOR_ERRORS_SIZE> ODRIVE_TORQUESENSOR_ERRORS;

    const size_t ODRIVE_CONTROLLER_ERRORS_SIZE = 11;
    extern const std::array<std::string, ODRIVE_CONTROLLER_ERRORS_SIZE> ODRIVE_CONTROLLER_ERRORS;
    

    // Add an error type to the description
    void addErrorToDescription(size_t index, ErrorRegister error_register, std::string& description);

    template <typename T> std::string parseError(T error, ErrorRegister error_register)
    {
        // TODO: constraint template types
        // https://gitlab.com/project-march/march/-/issues/982
        if (error == 0) {
            return "None. ";
        } else {
            std::string description;
            const auto size = sizeof(error) * CHAR_BIT;
            const std::bitset<size> bitset(error);

            for (size_t i = 0; i < size; i++) {
                if (bitset.test(i)) {
                    addErrorToDescription(i, error_register, description);
                }
            }
            return description;
        }
    }

} // namespace error
} // namespace march
#endif // MARCH_HARDWARE_MOTOR_CONTROLLER_ERROR_H
