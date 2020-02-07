// Copyright 2018 Project March.
#include "mocks/MockEncoder.cpp"

#include <march_hardware/error/hardware_exception.h>
#include <march_hardware/IMotionCube.h>

#include <gtest/gtest.h>

class IMotionCubeTest : public ::testing::Test
{
protected:
  march::Encoder encoder;
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
  ASSERT_THROW(imc.actuateRad(1), march::error::HardwareException);
}

TEST_F(IMotionCubeTest, ActuationModeTorqueActuateRad)
{
  march::IMotionCube imc(1, encoder, march::ActuationMode::torque);

  ASSERT_EQ(march::ActuationMode::torque, imc.getActuationMode().getValue());
  ASSERT_THROW(imc.actuateRad(1), march::error::HardwareException);
}

TEST_F(IMotionCubeTest, ActuationModePositionActuateTorque)
{
  march::IMotionCube imc(1, encoder, march::ActuationMode::position);

  ASSERT_THROW(imc.actuateTorque(1), march::error::HardwareException);
}
