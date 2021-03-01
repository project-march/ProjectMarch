// Copyright 2019 Project March.
#include "march_hardware/encoder/encoder.h"
#include "march_hardware/ethercat/pdo_types.h"
#include "march_hardware/error/hardware_exception.h"

namespace march
{
Encoder::Encoder(size_t number_of_bits) : total_positions_(Encoder::calculateTotalPositions(number_of_bits))
{
}

size_t Encoder::getTotalPositions() const
{
  return this->total_positions_;
}

size_t Encoder::calculateTotalPositions(size_t number_of_bits)
{
  if (number_of_bits < Encoder::MIN_RESOLUTION || number_of_bits > Encoder::MAX_RESOLUTION)
  {
    throw error::HardwareException(error::ErrorType::INVALID_ENCODER_RESOLUTION,
                                   "Encoder resolution of %d is not within range [%ld, %ld]", number_of_bits,
                                   Encoder::MIN_RESOLUTION, Encoder::MAX_RESOLUTION);
  }
  return 1u << number_of_bits;
}

}  // namespace march
