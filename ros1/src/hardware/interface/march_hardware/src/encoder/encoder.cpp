// Copyright 2019 Project March.
#include "march_hardware/encoder/encoder.h"
#include "march_hardware/error/hardware_exception.h"
#include "march_hardware/ethercat/pdo_types.h"

namespace march {
Encoder::Encoder(size_t number_of_bits)
    : total_positions_(Encoder::calculateTotalPositions(number_of_bits))
{
}

int32_t Encoder::getAngleIU(
    const PdoSlaveInterface& pdo, uint8_t byte_offset) const
{
    bit32 return_byte = pdo.read32(byte_offset);
    return return_byte.i;
}

double Encoder::getAngleRad(
    const PdoSlaveInterface& pdo, uint8_t byte_offset) const
{
    return this->toRad(this->getAngleIU(pdo, byte_offset));
}

double Encoder::getVelocityIU(
    const PdoSlaveInterface& pdo, uint8_t byte_offset) const
{
    bit32 return_byte = pdo.read32(byte_offset);
    return return_byte.i
        / (TIME_PER_VELOCITY_SAMPLE * FIXED_POINT_TO_FLOAT_CONVERSION);
}

double Encoder::getVelocityRad(
    const PdoSlaveInterface& pdo, uint8_t byte_offset) const
{
    return this->getVelocityIU(pdo, byte_offset) * this->getRadPerBit();
}

size_t Encoder::getTotalPositions() const
{
    return this->total_positions_;
}

size_t Encoder::calculateTotalPositions(size_t number_of_bits)
{
    if (number_of_bits < Encoder::MIN_RESOLUTION
        || number_of_bits > Encoder::MAX_RESOLUTION) {
        throw error::HardwareException(
            error::ErrorType::INVALID_ENCODER_RESOLUTION,
            "Encoder resolution of %d is not within range [%ld, %ld]",
            number_of_bits, Encoder::MIN_RESOLUTION, Encoder::MAX_RESOLUTION);
    }
    return 1u << number_of_bits;
}

} // namespace march
