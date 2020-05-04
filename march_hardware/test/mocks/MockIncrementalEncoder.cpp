#include "march_hardware/encoder/IncrementalEncoder.h"

#include <gmock/gmock.h>

class MockIncrementalEncoder : public march::IncrementalEncoder
{
public:
  MockIncrementalEncoder() : IncrementalEncoder(10, 100.0)
  {
  }

  MOCK_METHOD0(getAngleIU, int32_t());
  MOCK_METHOD0(getAngleRad, double());
};
