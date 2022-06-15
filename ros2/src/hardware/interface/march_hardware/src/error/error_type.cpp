// Copyright 2020 Project March.
#include "march_hardware/error/error_type.h"

namespace march {
namespace error {
    const char* getErrorDescription(ErrorType type)
    {
        switch (type) {
            case ErrorType::INVALID_ACTUATION_MODE:
                return "An invalid actuation mode was used to perform an "
                       "action";
            case ErrorType::INVALID_ACTUATE_POSITION:
                return "Invalid IU position command";
            case ErrorType::ENCODER_RESET:
                return "An encoder has reset and reads an incorrect value";
            case ErrorType::OUTSIDE_HARD_LIMITS:
                return "A joint is outside its defined hard limits";
            case ErrorType::TARGET_EXCEEDS_MAX_DIFFERENCE:
                return "The target position exceeds the max allowed difference "
                       "from the current position";
            case ErrorType::TARGET_TORQUE_EXCEEDS_MAX_TORQUE:
                return "The target torque exceeds the max allowed torque";
            case ErrorType::PDO_OBJECT_NOT_DEFINED:
                return "The to be added PDO object was not defined";
            case ErrorType::PDO_REGISTER_OVERFLOW:
                return "The PDO map could not fit within the registers";
            case ErrorType::WRITING_INITIAL_SETTINGS_FAILED:
                return "Failed to write initial settings to slave required for "
                       "operation";
            case ErrorType::NO_SOCKET_CONNECTION:
                return "Failed to connect to socket";
            case ErrorType::NOT_ALL_SLAVES_FOUND:
                return "Not all configured slaves were able to be found";
            case ErrorType::FAILED_TO_REACH_OPERATIONAL_STATE:
                return "At least one slave failed to reach ethercat "
                       "operational state";
            case ErrorType::INVALID_ENCODER_RESOLUTION:
                return "The encoder resolution is outside the allowed range";
            case ErrorType::INVALID_RANGE_OF_MOTION:
                return "The lower and upper limits of an encoder are "
                       "conflicting";
            case ErrorType::INVALID_SLAVE_CONFIGURATION:
                return "The slave configuration contains duplicate slave "
                       "indices";
            case ErrorType::NOT_ALLOWED_TO_ACTUATE:
                return "A joint is not allowed to actuate, yet it's trying to "
                       "actuate";
            case ErrorType::INVALID_SLAVE_INDEX:
                return "Slave index has an invalid value";
            case ErrorType::MISSING_URDF_JOINT:
                return "Required joint not defined in URDF";
            case ErrorType::MISSING_REQUIRED_KEY:
                return "Required robot config key not defined";
            case ErrorType::INIT_URDF_FAILED:
                return "Failed to load URDF from parameter server";
            case ErrorType::INVALID_SW_STRING:
                return "Slave has incorrect SW file";
            case ErrorType::SLAVE_LOST_TIMOUT:
                return "EtherCAT slave monitor timer elapsed, connection has "
                       "been lost";
            case ErrorType::ODRIVE_WRONG_AXIS_NUMBER:
                return "ODrive axis number must be either 0 or 1";
            case ErrorType::INVALID_MOTOR_CONTROLLER:
                return "MotorController must be IMotionCube or ODrive";
            case ErrorType::PREPARE_ACTUATION_ERROR:
                return "Something went wrong with preparing for actuation";
            case ErrorType::INVALID_ENCODER_DIRECTION:
                return "Encoder direction must be either -1 or 1";
            case ErrorType::BUSY_WAITING_FUNCTION_MAXIMUM_TRIES_REACHED:
                return "Busy waiting function reached maximum number of tries";
            default:
                return "Unknown error occurred. Please create/use a documented "
                       "error";
        }
    }

    std::ostream& operator<<(std::ostream& s, ErrorType type)
    {
        s << "E" << static_cast<int>(type) << ": " << getErrorDescription(type);
        return s;
    }
} // namespace error
} // namespace march
