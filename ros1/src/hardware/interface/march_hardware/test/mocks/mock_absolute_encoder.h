#pragma once
#include "march_hardware/encoder/absolute_encoder.h"

#include <gmock/gmock.h>

class MockAbsoluteEncoder : public march::AbsoluteEncoder
{
public:
  MockAbsoluteEncoder() : AbsoluteEncoder(10, 0, 162, 0, 1, 0.1, 0.9)
  {
  }
};
