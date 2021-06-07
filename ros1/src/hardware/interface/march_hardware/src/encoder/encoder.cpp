// Copyright 2019 Project March.
#include "march_hardware/encoder/encoder.h"
#include "march_hardware/error/hardware_exception.h"
#include "march_hardware/ethercat/pdo_types.h"
#include "march_hardware/motor_controller/motor_controller_type.h"

namespace march {
Encoder::Encoder(
    size_t resolution, MotorControllerType motor_controller_type, Direction direction)
    : total_positions_(Encoder::calculateTotalPositions(resolution))
    , motor_controller_type_(motor_controller_type)
    , direction_(direction)
{
}

size_t Encoder::getTotalPositions() const
{
    return this->total_positions_;
}

MotorControllerType Encoder::getMotorControllerType() const
{
    return motor_controller_type_;
}

Encoder::Direction Encoder::getDirection() const
{
    return direction_;
}

size_t Encoder::calculateTotalPositions(size_t resolution)
{
    if (resolution < Encoder::MIN_RESOLUTION
        || resolution > Encoder::MAX_RESOLUTION) {
        throw error::HardwareException(
            error::ErrorType::INVALID_ENCODER_RESOLUTION,
            "Encoder resolution of %d is not within range [%ld, %ld]",
            resolution, Encoder::MIN_RESOLUTION, Encoder::MAX_RESOLUTION);
    }
    return (size_t)1 << resolution;
}

double Encoder::positionIUToRadians(double position) const
{
    return position * getRadiansPerIU();
}

double Encoder::velocityIUToRadians(double velocity) const
{
    return velocity * getRadiansPerIU();
}

double Encoder::positionRadiansToIU(double position) const
{
    return position / getRadiansPerIU();
}

double Encoder::velocityRadiansToIU(double velocity) const
{
    return velocity / getRadiansPerIU();
}

} // namespace march
