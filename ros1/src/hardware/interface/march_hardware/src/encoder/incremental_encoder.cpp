// Copyright 2019 Project March.
#include "march_hardware/encoder/incremental_encoder.h"
#include "march_hardware/error/hardware_exception.h"
#include "march_hardware/motor_controller/motor_controller_type.h"

namespace march {
IncrementalEncoder::IncrementalEncoder(size_t resolution,
    MotorControllerType motor_controller_type, double transmission)
    : Encoder(resolution, motor_controller_type)
    , transmission_(transmission)
{
}

double IncrementalEncoder::getRadiansPerIU() const
{
    switch (getMotorControllerType()) {
        case MotorControllerType::IMotionCube:
            return PI_2 / (getTotalPositions() * transmission_);
        case MotorControllerType::ODrive:
            return PI_2 / transmission_;
        default:
            throw error::HardwareException(
                error::ErrorType::INVALID_MOTOR_CONTROLLER);
    }
}

double IncrementalEncoder::getTransmission() const
{
    return transmission_;
}
} //  namespace march
