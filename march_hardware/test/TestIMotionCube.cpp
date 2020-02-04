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
  march::IMotionCube imc = march::IMotionCube(1, encoder);
  ASSERT_EQ(1, imc.getSlaveIndex());
}

TEST_F(IMotionCubeDeathTest, SlaveIndexZero)
{
  ASSERT_DEATH(march::IMotionCube(0, encoder), "Slave configuration error: slaveindex 0 can not be smaller than "
                                               "1.");
}

TEST_F(IMotionCubeTest, SlaveIndexMinusOne)
{
  ASSERT_DEATH(march::IMotionCube(-1, encoder), "Slave configuration error: slaveindex -1 can not be smaller than "
                                                "1.");
}

TEST_F(IMotionCubeTest, NoSlaveIndexConstructor)
{
  ASSERT_NO_THROW(march::IMotionCube imc = march::IMotionCube());
}
TEST_F(IMotionCubeTest, NoSlaveIndexConstructorGetIndex)
{
  march::IMotionCube imc = march::IMotionCube();
  ASSERT_EQ(-1, imc.getSlaveIndex());
}

TEST_F(IMotionCubeTest, NoActuationMode)
{
  march::IMotionCube imc = march::IMotionCube(1, encoder);
  ASSERT_DEATH(imc.actuateRad(1), "trying to actuate rad, while actuationmode = unknown");
}

TEST_F(IMotionCubeTest, ActuationModeTorqueActuateRad)
{
  march::IMotionCube imc = march::IMotionCube(1, encoder);

  imc.setActuationMode(march::ActuationMode("torque"));
  ASSERT_DEATH(imc.actuateRad(1), "trying to actuate rad, while actuationmode = torque");
}

TEST_F(IMotionCubeTest, ActuationModePositionActuateTorque)
{
  march::IMotionCube imc = march::IMotionCube(1, encoder);

  imc.setActuationMode(march::ActuationMode("position"));
  ASSERT_DEATH(imc.actuateTorque(1), "trying to actuate torque, while actuationmode = position");
}

TEST_F(IMotionCubeTest, ChangeActuationModePosition)
{
  march::IMotionCube imc = march::IMotionCube(1, encoder);
  ASSERT_EQ(march::ActuationMode::unknown, imc.getActuationMode().getValue());

  imc.setActuationMode(march::ActuationMode("position"));
  ASSERT_EQ(march::ActuationMode::position, imc.getActuationMode().getValue());
}

TEST_F(IMotionCubeTest, ChangeActuationModeTorque)
{
  march::IMotionCube imc = march::IMotionCube(1, encoder);
  ASSERT_EQ(march::ActuationMode::unknown, imc.getActuationMode().getValue());

  imc.setActuationMode(march::ActuationMode("torque"));
  ASSERT_EQ(march::ActuationMode::torque, imc.getActuationMode().getValue());
}

TEST_F(IMotionCubeTest, ChangeActuationModeToUnknown)
{
  march::IMotionCube imc = march::IMotionCube(1, encoder);
  ASSERT_EQ(march::ActuationMode::unknown, imc.getActuationMode().getValue());

  imc.setActuationMode(march::ActuationMode("position"));
  ASSERT_EQ(march::ActuationMode::position, imc.getActuationMode().getValue());

  ASSERT_THROW(imc.setActuationMode(march::ActuationMode("unknown")), std::runtime_error);
}

TEST_F(IMotionCubeTest, ChangeActuationModeFromPositionToTorque)
{
  march::IMotionCube imc = march::IMotionCube(1, encoder);
  imc.setActuationMode(march::ActuationMode("position"));
  ASSERT_EQ(march::ActuationMode::position, imc.getActuationMode().getValue());

  ASSERT_THROW(imc.setActuationMode(march::ActuationMode("torque")), std::runtime_error);
}

TEST_F(IMotionCubeTest, ChangeActuationModeFromTorqueToPosition)
{
  march::IMotionCube imc = march::IMotionCube(1, encoder);
  imc.setActuationMode(march::ActuationMode("torque"));
  ASSERT_EQ(march::ActuationMode::torque, imc.getActuationMode().getValue());

  ASSERT_THROW(imc.setActuationMode(march::ActuationMode("position")), std::runtime_error);
}
