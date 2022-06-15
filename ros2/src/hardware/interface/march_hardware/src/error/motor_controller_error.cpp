// Copyright 2020 Project March.
#include "march_hardware/error/motor_controller_error.h"

namespace march {
namespace error {

    const std::array<std::string, IMOTIONCUBE_MOTION_ERRORS_SIZE>
        IMOTIONCUBE_MOTION_ERRORS = {
            "EtherCAT communication error. ",
            "Short-circuit. ",
            "Invalid setup (EEPROM) data. ",
            "Control error (position/speed error too big). ",
            "Communication error. ",
            "Motor position wraps around. ",
            "Positive limit switch. ",
            "Negative limit switch. ",
            "Over-current. ",
            "I2T protection. ",
            "Over-temperature motor. ",
            "Over-temperature drive. ",
            "Over-voltage. ",
            "Under-voltage. ",
            "Command error. ",
            "Drive disabled (Emergency button connector not shorted). ",
        };

    const std::array<std::string, IMOTIONCUBE_DETAILED_MOTION_ERRORS_SIZE>
        IMOTIONCUBE_DETAILED_MOTION_ERRORS = {
            "TML stack overflow. ",
            "TML stack underflow. ",
            "Homing not available. ",
            "Function not available. ",
            "UPD ignored. ",
            "Cancelable call ignored. ",
            "Positive software limit switch is active. ",
            "Negative software limit switch is active. ",
            "Invalid S-curve profile. ",
        };

    const std::array<std::string,
        IMOTIONCUBE_SECOND_DETAILED_MOTION_ERRORS_SIZE>
        IMOTIONCUBE_SECOND_DETAILED_MOTION_ERRORS = {
            "BiSS data CRC error. ",
            "BiSS data warning bit is set. ",
            "BiSS data error bit is set. ",
            "BiSS sensor missing. ",
            "Absolute Encoder Interface (AEI) interface error. ",
            "Hall sensor missing. ",
            "Position wraparound. The position 2^31 was exceeded. ",
        };

    const std::array<std::string, ODRIVE_AXIS_ERRORS_SIZE> ODRIVE_AXIS_ERRORS
        = {
              "Invalid state. ",
              "Watchdog timer expired. ",
              "Min endstop pressed. ",
              "Max endstop pressed. ",
              "Estop requested. ",
              "Homing without endstop. ",
              "Over temperature. ",
              "Unknown position. ",
          };

    const std::array<std::string, ODRIVE_MOTOR_ERRORS_SIZE> ODRIVE_MOTOR_ERRORS
        = {
              "Phase resistance out of range. ",
              "Phase inductance out of range. ",
              "DRV fault. ",
              "Control deadline missed. ",
              "Modulation magnitude. ",
              "Current sense saturation. ",
              "Current limit violation. ",
              "Modulation is NaN. ",
              "Motor thermistor over temperature. ",
              "FET thermistor over temperature. ",
              "Timer update missed. ",
              "Current measurement unavailable. ",
              "Controller failed. ",
              "I bus out of range. ",
              "Brake resistor disarmed. ",
              "System level. ",
              "Bad timing. ",
              "Unknown phase estimate. ",
              "Unknown phase velocity. ",
              "Unknown torque. ",
              "Unknown current command. ",
              "Unknown current measurement. ",
              "Unknown vbus voltage. ",
              "Unknown voltage command. ",
              "Unknown gains. ",
              "Controller initializing. ",
              "Unbalanced phases.",
          };

    const std::array<std::string, ODRIVE_ENCODER_ERRORS_SIZE>
        ODRIVE_ENCODER_ERRORS = {
            "Unstable gain. ",
            "CPR polepairs mismatch. ",
            "No response. ",
            "Unsupported encoder mode. ",
            "Illegal hall state. ",
            "Index not found yet. ",
            "ABS SPI timeout. ",
            "ABS SPI communication failure. ",
            "ABS SPI not ready. ",
            "Hall not calibrated yet. ",
        };

    const std::array<std::string, ODRIVE_DIEBOSLAVE_ERRORS_SIZE>
        ODRIVE_DIEBOSLAVE_ERRORS = {
            "Invalid axis. ",
            "Request state failed. ",
            "Motor not ready. ",
            "Motor not calibrated. ",
            "Encoder not ready. ",
            "Encoder index not yet found. ",
            "Encoder not precalibrated. ",
            "Encoder: absolute position not valid. ",
            "Encoder: absolute gpio pin not valid. ",
        };

    const std::array<std::string, ODRIVE_CONTROLLER_ERRORS_SIZE>
        ODRIVE_CONTROLLER_ERRORS = {
            "Overspeed. ",
            "Invalid input mode. ",
            "Unstable gain. ",
            "Invalid mirror axis. ",
            "Invalid load encoder. ",
            "Invalid estimate. ",
            "Invalid circular range. ",
            "Spinout detected. ",
        };

    void addErrorToDescription(
        size_t index, ErrorRegister error_register, std::string& description)
    {
        switch (error_register) {
            case ErrorRegister::IMOTIONCUBE_MOTION_ERROR:
                if (index < IMOTIONCUBE_MOTION_ERRORS.size()) {
                    description += IMOTIONCUBE_MOTION_ERRORS[index];
                }
                break;
            case ErrorRegister::IMOTIONCUBE_DETAILED_MOTION_ERROR:
                if (index < IMOTIONCUBE_DETAILED_MOTION_ERRORS.size()) {
                    description += IMOTIONCUBE_DETAILED_MOTION_ERRORS[index];
                }
                break;
            case ErrorRegister::IMOTIONCUBE_SECOND_DETAILED_MOTION_ERROR:
                if (index < IMOTIONCUBE_SECOND_DETAILED_MOTION_ERRORS.size()) {
                    description
                        += IMOTIONCUBE_SECOND_DETAILED_MOTION_ERRORS[index];
                }
                break;
            case ErrorRegister::ODRIVE_AXIS_ERROR:
                if (index < ODRIVE_AXIS_ERRORS.size()) {
                    description += ODRIVE_AXIS_ERRORS[index];
                }
                break;
            case ErrorRegister::ODRIVE_MOTOR_ERROR:
                if (index < ODRIVE_MOTOR_ERRORS.size()) {
                    description += ODRIVE_MOTOR_ERRORS[index];
                }
                break;
            case ErrorRegister::ODRIVE_ENCODER_ERROR:
                if (index < ODRIVE_MOTOR_ERRORS.size()) {
                    description += ODRIVE_ENCODER_ERRORS[index];
                }
                break;
            case ErrorRegister::ODRIVE_DIEBOSLAVE_ERROR:
                if (index < ODRIVE_DIEBOSLAVE_ERRORS.size()) {
                    description += ODRIVE_DIEBOSLAVE_ERRORS[index];
                }
                break;
            case ErrorRegister::ODRIVE_CONTROLLER_ERROR:
                if (index < ODRIVE_CONTROLLER_ERRORS.size()) {
                    description += ODRIVE_CONTROLLER_ERRORS[index];
                }
                break;
            default:
                description += "Unknown error. ";
                break;
        }
    }

    template std::string parseError<uint16_t>(uint16_t, ErrorRegister);
    template std::string parseError<uint32_t>(uint32_t, ErrorRegister);

} // namespace error
} // namespace march
