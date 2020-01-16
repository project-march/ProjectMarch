#include "gmock/gmock.h"  // Brings in Google Mock.
#include "march_hardware/Encoder.h"

class MockEncoder : public march::Encoder
{
public:
  MOCK_METHOD0(getAngleDeg, float());
  MOCK_METHOD0(getAngleRad, float());
  MOCK_METHOD0(getAngle, float());
};
