#pragma once
#include "march_hardware/encoder/incremental_encoder.h"

#include <gmock/gmock.h>

class MockIncrementalEncoder : public march::IncrementalEncoder {
public:
    MockIncrementalEncoder()
        : IncrementalEncoder(/*number_of_bits=*/10, /*transmission=*/100.0)
    {
    }
};
