// Copyright 2019 Project March.
#ifndef MARCH_HARDWARE_ODRIVE_STATE_H
#define MARCH_HARDWARE_ODRIVE_STATE_H

#include "march_hardware/error/motor_controller_error.h"
#include "march_hardware/motor_controller/motor_controller_state.h"
#include <sstream>
#include <string>

namespace march {

enum class ODriveAxis { Zero = 0, One = 1 };

class ODriveAxisState {
public:
    enum Value : int8_t {
        UNDEFINED = 0,
        IDLE = 1,
        STARTUP_SEQUENCE = 2,
        FULL_CALIBRATION_SEQUENCE = 3,
        MOTOR_CALIBRATION = 4,
        SENSORLESS_CONTROL = 5,
        ENCODER_INDEX_SEARCH = 6,
        ENCODER_OFFSET_CALIBRATION = 7,
        CLOSED_LOOP_CONTROL = 8,
        LOCKIN_SPIN = 9,
        ENCODER_DIR_FIND = 10,
        HOMING = 11,
        ENCODER_HALL_POLARITY_CALIBRATION = 12,
        ENCODER_HALL_PHASE_CALIBRATION = 13
    };

    ODriveAxisState()
        : value_(UNDEFINED)
    {
    }

    // NOLINTNEXTLINE(hicpp-explicit-conversions)
    ODriveAxisState(int8_t state)
        : value_(Value(state))
    {
    }

    // NOLINTNEXTLINE(hicpp-explicit-conversions)
    ODriveAxisState(Value value)
        : value_(value)
    {
    }

    std::string toString()
    {
        return toString(value_);
    }

    static std::string toString(Value value)
    {
        switch (value) {
            case UNDEFINED:
                return "Undefined";
            case IDLE:
                return "Idle";
            case STARTUP_SEQUENCE:
                return "Startup sequence";
            case FULL_CALIBRATION_SEQUENCE:
                return "Full calibration sequence";
            case MOTOR_CALIBRATION:
                return "Motor calibration";
            case SENSORLESS_CONTROL:
                return "Sensorless control";
            case ENCODER_INDEX_SEARCH:
                return "Encoder index search";
            case ENCODER_OFFSET_CALIBRATION:
                return "Encoder offset calibration";
            case CLOSED_LOOP_CONTROL:
                return "Close loop control";
            case LOCKIN_SPIN:
                return "Lockin spin";
            case ENCODER_DIR_FIND:
                return "Encoder dir find";
            case HOMING:
                return "Homing";
            case ENCODER_HALL_POLARITY_CALIBRATION:
                return "Encoder hall polarity calibration";
            case ENCODER_HALL_PHASE_CALIBRATION:
                return "Encoder hall phase calibration";
            default:
                return "Unrecognized ODrive axis state";
        }
    }

    bool operator==(Value v) const
    {
        return value_ == v;
    }
    bool operator==(ODriveAxisState a) const
    {
        return value_ == a.value_;
    }
    bool operator!=(ODriveAxisState a) const
    {
        return value_ != a.value_;
    }

    Value value_;
};

class ODriveState : public MotorControllerState {
public:
    ODriveState() = default;

    bool isOperational() override
    {
        return axis_state_.value_ == ODriveAxisState::CLOSED_LOOP_CONTROL;
    }

    bool hasError() override
    {
        return axis_error_ || motor_error_ || encoder_error_
            || encoder_manager_error_ || controller_error_;
    }

    std::optional<std::string> getErrorStatus() override
    {
        if (hasError()) {
            std::ostringstream error_stream;
            error_stream
                << "Axis: "
                << error::parseError(
                       axis_error_, error::ErrorRegister::ODRIVE_AXIS_ERROR)
                << ", "
                << "\nMotor: "
                << error::parseError(
                       motor_error_, error::ErrorRegister::ODRIVE_MOTOR_ERROR)
                << ", "
                << "\nEncoder: "
                << error::parseError(encoder_error_,
                       error::ErrorRegister::ODRIVE_ENCODER_ERROR)
                << ", "
                << "\nEncoder Manager: "
                << error::parseError(encoder_manager_error_,
                       error::ErrorRegister::ODRIVE_ENCODER_MANAGER_ERROR)
                << ", "
                << "\nController: "
                << error::parseError(controller_error_,
                       error::ErrorRegister::ODRIVE_CONTROLLER_ERROR);
            return error_stream.str();
        } else {
            return std::nullopt;
        }
    }

    std::string getOperationalState() override
    {
        return axis_state_.toString();
    }

    ODriveAxisState axis_state_;
    uint32_t axis_error_ {};
    uint32_t motor_error_ {};
    uint32_t encoder_error_ {};
    uint32_t encoder_manager_error_ {};
    uint32_t controller_error_ {};
};

} // namespace march

#endif // MARCH_HARDWARE_ODRIVE_STATE_OF_OPERATION_H
