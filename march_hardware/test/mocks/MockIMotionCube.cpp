#include "gmock/gmock.h"  // Brings in Google Mock.
#include "march_hardware/IMotionCube.h"

class MockIMotionCube : public march::IMotionCube
{
public:
  MOCK_METHOD0(getAngle, float());
};
