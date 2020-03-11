// Copyright 2019 Project March.
#include "march_hardware/encoder/IncrementalEncoder.h"

namespace march
{
IncrementalEncoder::IncrementalEncoder(size_t number_of_bits, double transmission)
  : Encoder(number_of_bits), transmission_(transmission)
{
}

double IncrementalEncoder::toRad(int32_t iu) const
{
  return iu * this->transmission_ * PI_2 / this->getTotalPositions();
}
}  //  namespace march
