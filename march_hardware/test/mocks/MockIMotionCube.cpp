#include "gmock/gmock.h"  // Brings in Google Mock.
#include "march_hardware/IMotionCube.h"

#include "MockEncoder.cpp"

class MockIMotionCube : public march::IMotionCube
{
public:
  MockIMotionCube() : march::IMotionCube(1, MockEncoder(), march::ActuationMode::unknown)
  {
  }
  MOCK_METHOD0(getAngle, float());
};
