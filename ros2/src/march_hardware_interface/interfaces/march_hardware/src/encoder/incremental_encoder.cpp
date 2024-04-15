// Copyright 2019 Project March.
#include "march_hardware/encoder/incremental_encoder.h"
#include "march_hardware/error/hardware_exception.h"
#include "march_hardware/motor_controller/motor_controller_type.h"

namespace march {
IncrementalEncoder::IncrementalEncoder(
    size_t counts_per_rotation, MotorControllerType motor_controller_type, Direction direction, double transmission)
    : Encoder(counts_per_rotation, motor_controller_type, direction)
    , transmission_(transmission)
{
    radians_per_iu_ = calculateRadiansPerIU();
}

IncrementalEncoder::IncrementalEncoder(
    size_t counts_per_rotation, MotorControllerType motor_controller_type, double transmission)
    : IncrementalEncoder(counts_per_rotation, motor_controller_type, Direction::Positive, transmission)
{
}

double IncrementalEncoder::calculateRadiansPerIU() const
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
        default:
            throw error::HardwareException(error::ErrorType::INVALID_MOTOR_CONTROLLER);
    }
}

double IncrementalEncoder::velocityRadiansToIU(double velocity) const
{
    switch (getMotorControllerType()) {
        case MotorControllerType::ODrive:
            return velocity / (PI_2 / transmission_);
        default:
            throw error::HardwareException(error::ErrorType::INVALID_MOTOR_CONTROLLER);
    }
}
} //  namespace march
