#include "MockAbsoluteEncoder.cpp"
#include "MockIncrementalEncoder.cpp"

#include "march_hardware/IMotionCube.h"

#include <memory>

#include <gmock/gmock.h>
#include <fstream>

class MockIMotionCube : public march::IMotionCube
{
private:
  std::string sw_stream_empty_;

public:
  MockIMotionCube()
    : IMotionCube(1, std::make_unique<MockAbsoluteEncoder>(), std::make_unique<MockIncrementalEncoder>(),
                  sw_stream_empty_, march::ActuationMode::unknown)
  {
  }

  MOCK_METHOD1(writeInitialSDOs, void(int));
  MOCK_METHOD0(goToOperationEnabled, void());

  MOCK_METHOD1(actuateRad, void(double));
  MOCK_METHOD1(actuateTorque, void(int16_t));
};
