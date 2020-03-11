#include "march_hardware/encoder/AbsoluteEncoder.h"

#include <gmock/gmock.h>

class MockAbsoluteEncoder : public march::AbsoluteEncoder
{
public:
  MockAbsoluteEncoder() : AbsoluteEncoder(10, 0, 162, 0, 1, 0.1, 0.9)
  {
  }

  MOCK_METHOD0(getAngleIU, int32_t());
  MOCK_METHOD0(getAngleRad, double());
};
