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
    enum Value : uint32_t {
        UNDEFINED = 0,
        IDLE = 1,
        STARTUP_SEQUENCE = 2,
        FULL_CALIBRATION_SEQUENCE = 3,
        MOTOR_CALIBRATION = 4,
        ENCODER_INDEX_SEARCH = 6,
        ENCODER_OFFSET_CALIBRATION = 7,
        CLOSED_LOOP_CONTROL = 8,
        LOCKIN_SPIN = 9,
        ENCODER_DIR_FIND = 10,
        HOMING = 11,
        ENCODER_HALL_POLARITY_CALIBRATION = 12,
        ENCODER_HALL_PHASE_CALIBRATION = 13,

        // Custom MARCH-made request-only states
        // This will execute functions on the DieBoSlave while the ODrive stays
        // in Idle state
        CLEAR_ODRIVE_ERRORS = 32,
        CLEAR_DIEBOSLAVE_ERRORS = 33,
        CLEAR_ALL_ERRORS = 34
    };

    ODriveAxisState()
        : value_(UNDEFINED)
    {
    }

    // NOLINTNEXTLINE(hicpp-explicit-conversions)
    ODriveAxisState(int32_t state)
        : value_(Value(state))
    {
    }

    // NOLINTNEXTLINE(hicpp-explicit-conversions)
    ODriveAxisState(Value value)
        : value_(value)
    {
    }

    std::string toString() const
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
            case ENCODER_INDEX_SEARCH:
                return "Encoder index search";
            case ENCODER_OFFSET_CALIBRATION:
                return "Encoder offset calibration";
            case CLOSED_LOOP_CONTROL:
                return "Closed loop control";
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

    bool dataIsValid() const override
    {
        return axis_state_.value_ != ODriveAxisState::UNDEFINED;
    }

    bool isOperational() const override
    {
        return axis_state_.value_ == ODriveAxisState::CLOSED_LOOP_CONTROL;
    }

    bool hasError() const override
    {
        return !isOperational();
        // axis_error_ || motor_error_ || encoder_error_
        //    || encoder_manager_error_ || controller_error_;
    }

    std::optional<std::string> getErrorStatus() const override
    {
        if (hasError()) {
            std::ostringstream error_stream;
            error_stream << "State: " << axis_state_.toString() << " ("
                         << axis_state_.value_ << ")" << std::endl
                         << "Axis: "
                         << error::parseError(axis_error_,
                                error::ErrorRegister::ODRIVE_AXIS_ERROR)
                         << std::endl
                         << "Motor: "
                         << error::parseError(motor_error_,
                                error::ErrorRegister::ODRIVE_MOTOR_ERROR)
                         << std::endl
                         << "Encoder: "
                         << error::parseError(encoder_error_,
                                error::ErrorRegister::ODRIVE_ENCODER_ERROR)
                         << std::endl
                         << "DieBOSlave: "
                         << error::parseError(dieboslave_error_,
                                error::ErrorRegister::ODRIVE_DIEBOSLAVE_ERROR)
                         << std::endl
                         << "Controller: "
                         << error::parseError(controller_error_,
                                error::ErrorRegister::ODRIVE_CONTROLLER_ERROR);
            return error_stream.str();
        } else {
            return std::nullopt;
        }
    }

    std::string getOperationalState() const override
    {
        return axis_state_.toString();
    }

    ODriveAxisState axis_state_;
    uint32_t axis_error_ {};
    uint32_t motor_error_ {};
    uint32_t encoder_error_ {};
    uint32_t dieboslave_error_ {};
    uint32_t controller_error_ {};
};

} // namespace march

#endif // MARCH_HARDWARE_ODRIVE_STATE_H
