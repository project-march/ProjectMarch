#pragma once
#include "mock_absolute_encoder.h"
#include "mock_incremental_encoder.h"
#include "mock_slave.h"

#include "march_hardware/motor_controller/imotioncube/imotioncube.h"
#include "march_hardware/motor_controller/imotioncube/imotioncube_state.h"
#include "march_hardware/ethercat/sdo_interface.h"

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

  MOCK_METHOD0(getState, std::unique_ptr<march::MotorControllerState>());

  MOCK_METHOD0(prepareActuation, void());

  MOCK_METHOD0(isIncrementalEncoderMorePrecise, bool());

  MOCK_METHOD0(getIncrementalPositionUnchecked, float());
  MOCK_METHOD0(getAbsolutePositionUnchecked, float());
  MOCK_METHOD0(getIncrementalVelocityUnchecked, float());
  MOCK_METHOD0(getAbsoluteVelocityUnchecked, float());

  MOCK_METHOD0(getMotorControllerVoltage, float());
  MOCK_METHOD0(getMotorVoltage, float());
  MOCK_METHOD0(getMotorCurrent, float());

  MOCK_METHOD1(actuateRadians, void(float));
  MOCK_METHOD1(actuateTorque, void(float));

  MOCK_METHOD2(initSdo, bool(march::SdoSlaveInterface& sdo, int cycle_time));
  MOCK_METHOD1(reset, void(march::SdoSlaveInterface&));
};
