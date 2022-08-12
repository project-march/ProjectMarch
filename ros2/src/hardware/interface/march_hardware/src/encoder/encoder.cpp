// Copyright 2019 Project March.
#include "march_hardware/encoder/encoder.h"
#include "march_hardware/error/hardware_exception.h"
#include "march_hardware/ethercat/pdo_types.h"
#include "march_hardware/motor_controller/motor_controller_type.h"

namespace march {
Encoder::Encoder(size_t counts_per_rotation, MotorControllerType motor_controller_type, Direction direction)
    : total_positions_(Encoder::calculateTotalPositions(counts_per_rotation))
    , motor_controller_type_(motor_controller_type)
    , direction_(direction)
{
}

Encoder::Encoder(size_t counts_per_rotation, MotorControllerType motor_controller_type)
    : Encoder(counts_per_rotation, motor_controller_type, Direction::Positive)
{
}

size_t Encoder::getTotalPositions() const
{
    return this->total_positions_;
}

double Encoder::getRadiansPerIU() const
{
    return radians_per_iu_;
}

MotorControllerType Encoder::getMotorControllerType() const
{
    return motor_controller_type_;
}

Encoder::Direction Encoder::getDirection() const
{
    return direction_;
}

size_t Encoder::calculateTotalPositions(size_t counts_per_rotation)
{
    if (counts_per_rotation < Encoder::MIN_COUNTS_PER_ROTATION
        || counts_per_rotation > Encoder::MAX_COUNTS_PER_ROTATION) {
        throw error::HardwareException(error::ErrorType::INVALID_ENCODER_COUNTS_PER_ROTATION,
            "Encoder CPR (counts per rotation) of %d is not within range [%ld, %ld]", counts_per_rotation,
            Encoder::MIN_COUNTS_PER_ROTATION, Encoder::MAX_COUNTS_PER_ROTATION);
    }
    return counts_per_rotation;
}

double Encoder::positionIUToRadians(double position) const
{
    return position * getRadiansPerIU();
}

double Encoder::velocityIUToRadians(double velocity) const
{
    return velocity * getRadiansPerIU();
}

int32_t Encoder::positionRadiansToIU(double position) const
{
    return position / getRadiansPerIU();
}

double Encoder::velocityRadiansToIU(double velocity) const
{
    return velocity / getRadiansPerIU();
}

} // namespace march
