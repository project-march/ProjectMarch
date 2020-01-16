// Copyright 2018 Project March.

#include "gtest/gtest.h"
#include <gmock/gmock.h>
#include "march_hardware/IMotionCube.h"
#include "mocks/MockEncoder.cpp"

using ::testing::AtLeast;
using ::testing::Return;

class IMotionCubeTest : public ::testing::Test
{
protected:
  march::Encoder encoder;
  march::ActuationMode actuationMode;
  void SetUp() override
  {
    encoder = march::Encoder();
  }
};

class IMotionCubeDeathTest : public IMotionCubeTest
{
};

TEST_F(IMotionCubeTest, SlaveIndexOne)
{
  march::IMotionCube imc(1, encoder, march::ActuationMode::unknown);
  ASSERT_EQ(1, imc.getSlaveIndex());
}

TEST_F(IMotionCubeDeathTest, SlaveIndexZero)
{
  ASSERT_DEATH(march::IMotionCube(0, encoder, march::ActuationMode::unknown), "Slave configuration error: slaveindex 0 "
                                                                              "can not be smaller than "
                                                                              "1.");
}

TEST_F(IMotionCubeTest, SlaveIndexMinusOne)
{
  ASSERT_DEATH(march::IMotionCube(-1, encoder, march::ActuationMode::unknown), "Slave configuration error: slaveindex "
                                                                               "-1 can not be smaller than "
                                                                               "1.");
}

TEST_F(IMotionCubeTest, NoActuationMode)
{
  march::IMotionCube imc(1, encoder, march::ActuationMode::unknown);
  ASSERT_DEATH(imc.actuateRad(1), "trying to actuate rad, while actuationmode = unknown");
}

TEST_F(IMotionCubeTest, ActuationModeTorqueActuateRad)
{
  march::IMotionCube imc(1, encoder, march::ActuationMode::torque);

  ASSERT_EQ(march::ActuationMode::torque, imc.getActuationMode().getValue());
  ASSERT_DEATH(imc.actuateRad(1), "trying to actuate rad, while actuationmode = torque");
}

TEST_F(IMotionCubeTest, ActuationModePositionActuateTorque)
{
  march::IMotionCube imc(1, encoder, march::ActuationMode::position);

  ASSERT_DEATH(imc.actuateTorque(1), "trying to actuate torque, while actuationmode = position");
}
