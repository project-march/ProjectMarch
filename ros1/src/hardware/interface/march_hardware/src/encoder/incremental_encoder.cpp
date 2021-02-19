// Copyright 2019 Project March.
#include "march_hardware/encoder/incremental_encoder.h"

namespace march
{
IncrementalEncoder::IncrementalEncoder(size_t number_of_bits, double transmission)
  : Encoder(number_of_bits), transmission_(transmission)
{
}

double IncrementalEncoder::getRadiansPerBit() const
{
  return PI_2 / (getTotalPositions() * this->transmission_);
}

double IncrementalEncoder::toIU(double radians, bool /*is_position*/) const
{
  return radians / getRadiansPerBit();
}

double IncrementalEncoder::toRadians(double iu, bool /*is_position*/) const
{
  return iu * getRadiansPerBit();
}

double IncrementalEncoder::getTransmission() const
{
  return this->transmission_;
}
}  //  namespace march
