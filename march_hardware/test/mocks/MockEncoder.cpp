#include "gmock/gmock.h"  // Brings in Google Mock.
#include "march_hardware/Encoder.h"

class MockEncoder : public march::Encoder
{
public:
  MockEncoder() : Encoder(10, 0, 162, 0, 1, 0.1, 0.9)
  {
  }

  MOCK_METHOD0(getAngleIU, int32_t());
  MOCK_METHOD0(getAngleRad, double());
};
