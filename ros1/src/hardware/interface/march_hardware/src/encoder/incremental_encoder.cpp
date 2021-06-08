// Copyright 2019 Project March.
#include "march_hardware/encoder/incremental_encoder.h"
#include "march_hardware/error/hardware_exception.h"
#include "march_hardware/motor_controller/motor_controller_type.h"

namespace march {
IncrementalEncoder::IncrementalEncoder(size_t resolution,
    MotorControllerType motor_controller_type, Direction direction,
    double transmission)
    : Encoder(resolution, motor_controller_type, direction)
    , transmission_(transmission)
{
}

IncrementalEncoder::IncrementalEncoder(size_t resolution,
    MotorControllerType motor_controller_type, double transmission)
    : IncrementalEncoder(
        resolution, motor_controller_type, Direction::Positive, transmission)
{
}

double IncrementalEncoder::getRadiansPerIU() const
{
    return PI_2 / (getTotalPositions() * transmission_);
}

double IncrementalEncoder::getTransmission() const
{
    return transmission_;
}

double IncrementalEncoder::velocityIUToRadians(double velocity) const
{
    switch (getMotorControllerType()) {
        case MotorControllerType::ODrive:
            return velocity * (PI_2 / transmission_);
        case MotorControllerType::IMotionCube:
            return Encoder::velocityIUToRadians(velocity);
        default:
            throw error::HardwareException(
                error::ErrorType::INVALID_MOTOR_CONTROLLER);
    }
}

double IncrementalEncoder::velocityRadiansToIU(double velocity) const
{
    switch (getMotorControllerType()) {
        case MotorControllerType::ODrive:
            return velocity / (PI_2 / transmission_);
        case MotorControllerType::IMotionCube:
            return Encoder::velocityIUToRadians(velocity);
        default:
            throw error::HardwareException(
                error::ErrorType::INVALID_MOTOR_CONTROLLER);
    }
}
} //  namespace march
