#pragma once
#include "MockAbsoluteEncoder.h"
#include "MockIncrementalEncoder.h"
#include "MockSlave.h"

#include "march_hardware/IMotionCube.h"
#include "march_hardware/EtherCAT/sdo_interface.h"

#include <memory>

#include <gmock/gmock.h>

class MockIMotionCube : public march::IMotionCube
{
public:
  MockIMotionCube()
    : IMotionCube(MockSlave(), std::make_unique<MockAbsoluteEncoder>(), std::make_unique<MockIncrementalEncoder>(),
                  march::ActuationMode::unknown)
  {
  }

  MOCK_METHOD0(goToOperationEnabled, void());

  MOCK_METHOD0(getAngleRadIncremental, double());
  MOCK_METHOD0(getAngleRadAbsolute, double());

  MOCK_METHOD0(getVelocityRadIncremental, double());
  MOCK_METHOD0(getVelocityRadAbsolute, double());

  MOCK_METHOD0(getIMCVoltage, float());
  MOCK_METHOD0(getMotorVoltage, float());
  MOCK_METHOD0(getMotorCurrent, float());

  MOCK_METHOD1(actuateRad, void(double));
  MOCK_METHOD1(actuateTorque, void(int16_t));

  MOCK_METHOD2(initSdo, bool(march::SdoSlaveInterface& sdo, int cycle_time));
  MOCK_METHOD1(reset, void(march::SdoSlaveInterface&));
};
