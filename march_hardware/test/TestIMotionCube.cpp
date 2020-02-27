// Copyright 2018 Project March.
#include "mocks/MockEncoder.cpp"

#include <march_hardware/error/hardware_exception.h>
#include <march_hardware/IMotionCube.h>

#include <gtest/gtest.h>

class IMotionCubeTest : public ::testing::Test
{
protected:
  MockEncoder mock_encoder;
};

TEST_F(IMotionCubeTest, SlaveIndexOne)
{
  march::IMotionCube imc(1, this->mock_encoder, march::ActuationMode::unknown);
  ASSERT_EQ(1, imc.getSlaveIndex());
}

TEST_F(IMotionCubeTest, SlaveIndexZero)
{
  ASSERT_DEATH(march::IMotionCube(0, this->mock_encoder, march::ActuationMode::unknown), "Slave configuration error: "
                                                                                         "slaveindex 0 "
                                                                                         "can not be smaller than "
                                                                                         "1.");
}

TEST_F(IMotionCubeTest, SlaveIndexMinusOne)
{
  ASSERT_DEATH(march::IMotionCube(-1, this->mock_encoder, march::ActuationMode::unknown), "Slave configuration error: "
                                                                                          "slaveindex "
                                                                                          "-1 can not be smaller than "
                                                                                          "1.");
}

TEST_F(IMotionCubeTest, NoActuationMode)
{
  march::IMotionCube imc(1, this->mock_encoder, march::ActuationMode::unknown);
  ASSERT_THROW(imc.actuateRad(1), march::error::HardwareException);
}

TEST_F(IMotionCubeTest, ActuationModeTorqueActuateRad)
{
  march::IMotionCube imc(1, this->mock_encoder, march::ActuationMode::torque);

  ASSERT_EQ(march::ActuationMode::torque, imc.getActuationMode().getValue());
  ASSERT_THROW(imc.actuateRad(1), march::error::HardwareException);
}

TEST_F(IMotionCubeTest, ActuationModePositionActuateTorque)
{
  march::IMotionCube imc(1, this->mock_encoder, march::ActuationMode::position);

  ASSERT_THROW(imc.actuateTorque(1), march::error::HardwareException);
}
