// Copyright 2019 Project March.
#include "march_hardware/encoder/incremental_encoder.h"

namespace march {
IncrementalEncoder::IncrementalEncoder(
    size_t number_of_bits, double transmission)
    : Encoder(number_of_bits)
    , transmission_(transmission)
{
}

double IncrementalEncoder::getRadiansPerIU() const
{
    return PI_2 / (getTotalPositions() * this->transmission_);
}

double IncrementalEncoder::getTransmission() const
{
    return this->transmission_;
}
} //  namespace march
