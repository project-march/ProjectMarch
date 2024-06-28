/** Copyright 2020 Project March.
 *  These errors are based on the odrive errors check:
 *  https://gitlab.com/project-march/hardware/ODrive/-/blob/devel-0.5.2/tools/odrive/enums.py
 */
#include "march_hardware/error/motor_controller_error.h"

namespace march {
namespace error {

    const std::array<std::string, ODRIVE_ERRORS_SIZE> ODRIVE_ERRORS = {
        "Control iteration missed. ",
        "DC bus under voltage. ",
        "DC bus over voltage (replace resistor).",
        "DC bus over regen current. ",
        "DC bus over current. ",
        "Brake deadtime violation. ",
        "Brake duty cycle nan. ",
        "Invalid brake resistance. ",
    };

    const std::array<std::string, ODRIVE_AXIS_ERRORS_SIZE> ODRIVE_AXIS_ERRORS = {
        "Invalid state. ",
        "Dummy error 2. ",
        "Dummy error 4. ",
        "Dummy error 8. ",
        "Dummy error 10. ",
        "Dummy error 20. ",        
        "Motor failed. ",
        "Sensorless estimator failed. ",
        "Encoder failed. ",
        "Controller failed. ",
        "Watchdog timer expired. ",
        "Min endstop pressed. ",
        "Max endstop pressed. ",
        "Estop requested. ",
        "Dummy error 4000. ",
        "Dummy error 8000. ",
        "Dummy error 10000. ",
        "Homing without endstop. ",
        "Over temperature. ",
        "Unknown position. ",
        "Unknown moment. ",
    };

    const std::array<std::string, ODRIVE_MOTOR_ERRORS_SIZE> ODRIVE_MOTOR_ERRORS = {
        "Phase resistance out of range. ",
        "Phase inductance out of range. ",
        "Dummy error 4. ",
        "DRV fault. ",
        "Control deadline missed. ",
        "Dummy error 20. ", 
        "Dummy error 40. ", 
        "Modulation magnitude. ",
        "Dummy error 100. ", 
        "Dummy error 200. ", 
        "Current sense saturation. ",
        "Dummy error 800. ", 
        "Current limit violation (replace/check incremental encoder cable + recalibrate). ",
        "Dummy error 2000. ", 
        "Dummy error 4000. ", 
        "Dummy error 8000. ", 
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
        "I2T limit violation.",
    };

    const std::array<std::string, ODRIVE_ENCODER_ERRORS_SIZE> ODRIVE_ENCODER_ERRORS = {
        "Unstable gain. ",
        "CPR polepairs mismatch (calibrate, or replace encoder). ",
        "No response. ",
        "Unsupported encoder mode. ",
        "Illegal hall state. ",
        "Index not found yet. ",
        "ABS SPI timeout. ",
        "ABS SPI communication failure. ",
        "ABS SPI not ready. ",
        "Hall not calibrated yet. ",
        "Unconnected cable. ",
        "Unconnected AIE cable. ",
        "Unsupported AIE encoder mode.",
    };

    const std::array<std::string, ODRIVE_TORQUESENSOR_ERRORS_SIZE> ODRIVE_TORQUESENSOR_ERRORS = {
        "Dummy error. ",
        "Torque limit exceeded. ",
        "Time limit exceeded. ",
    };

    const std::array<std::string, ODRIVE_CONTROLLER_ERRORS_SIZE> ODRIVE_CONTROLLER_ERRORS = {
        "Overspeed. ",
        "Invalid input mode. ",
        "Unstable gain. ",
        "Invalid mirror axis. ",
        "Invalid load encoder. ",
        "Invalid estimate. ",
        "Invalid circular range. ",
        "Spinout detected (recalibrate, or replacing motor cable + recalibrate). ",
        "Invalid weights. ",
        "Target position delta too high. ",
        "Controller moving in the wrong direction.",
    };


    void addErrorToDescription(size_t index, ErrorRegister error_register, std::string& description)
    {
        switch (error_register) {
            case ErrorRegister::ODRIVE_ERROR:
                if (index < ODRIVE_ERRORS.size()) {
                    description += ODRIVE_ERRORS[index];
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
                if (index < ODRIVE_ENCODER_ERRORS.size()) {
                    description += ODRIVE_ENCODER_ERRORS[index];
                }
                break;
            case ErrorRegister::ODRIVE_TORQUESENSOR_ERROR:
                if (index < ODRIVE_TORQUESENSOR_ERRORS.size()) {
                    description += ODRIVE_TORQUESENSOR_ERRORS[index];
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
    template std::string parseError<uint64_t>(uint64_t, ErrorRegister);

} // namespace error
} // namespace march
