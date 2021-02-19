#pragma once
#include "march_hardware/encoder/incremental_encoder.h"

#include <gmock/gmock.h>

class MockIncrementalEncoder : public march::IncrementalEncoder
{
public:
  MockIncrementalEncoder() : IncrementalEncoder(10, 100.0)
  {
  }
};
