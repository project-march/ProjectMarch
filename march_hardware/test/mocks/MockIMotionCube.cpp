#include "MockAbsoluteEncoder.cpp"
#include "MockIncrementalEncoder.cpp"

#include "march_hardware/IMotionCube.h"

#include <memory>

#include <gmock/gmock.h>

class MockIMotionCube : public march::IMotionCube
{
public:
  MockIMotionCube()
    : IMotionCube(1, std::make_unique<MockAbsoluteEncoder>(), std::make_unique<MockIncrementalEncoder>(),
                  march::ActuationMode::unknown)
  {
  }

  MOCK_METHOD1(writeInitialSDOs, void(int));
  MOCK_METHOD0(goToOperationEnabled, void());
  MOCK_METHOD0(reset, void());

  MOCK_METHOD1(actuateRad, void(double));
  MOCK_METHOD1(actuateTorque, void(int16_t));
};
