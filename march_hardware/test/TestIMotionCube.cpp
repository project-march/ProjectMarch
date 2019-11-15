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
  march4cpp::Encoder encoder;
  march4cpp::ActuationMode actuationMode;
  void SetUp() override
  {
    encoder = march4cpp::Encoder();
  }
};

class IMotionCubeDeathTest : public IMotionCubeTest
{
};

TEST_F(IMotionCubeTest, SlaveIndexOne)
{
  march4cpp::IMotionCube imc = march4cpp::IMotionCube(1, encoder);
  ASSERT_EQ(1, imc.getSlaveIndex());
}

TEST_F(IMotionCubeDeathTest, SlaveIndexZero)
{
  ASSERT_DEATH(march4cpp::IMotionCube(0, encoder), "Slave configuration error: slaveindex 0 can not be smaller than "
                                                   "1.");
}

TEST_F(IMotionCubeTest, SlaveIndexMinusOne)
{
  ASSERT_DEATH(march4cpp::IMotionCube(-1, encoder), "Slave configuration error: slaveindex -1 can not be smaller than "
                                                    "1.");
}

TEST_F(IMotionCubeTest, NoSlaveIndexConstructor)
{
  ASSERT_NO_THROW(march4cpp::IMotionCube imc = march4cpp::IMotionCube());
}
TEST_F(IMotionCubeTest, NoSlaveIndexConstructorGetIndex)
{
  march4cpp::IMotionCube imc = march4cpp::IMotionCube();
  ASSERT_EQ(-1, imc.getSlaveIndex());
}

TEST_F(IMotionCubeTest, NoActuationMode)
{
  march4cpp::IMotionCube imc = march4cpp::IMotionCube(1, encoder);
  ASSERT_DEATH(imc.actuateRad(1), "trying to actuate rad, while actuationmode = unknown");
}

TEST_F(IMotionCubeTest, ActuationModeTorqueActuateRad)
{
  march4cpp::IMotionCube imc = march4cpp::IMotionCube(1, encoder);

  imc.setActuationMode(march4cpp::ActuationMode("torque"));
  ASSERT_DEATH(imc.actuateRad(1), "trying to actuate rad, while actuationmode = torque");
}

TEST_F(IMotionCubeTest, ActuationModePositionActuateTorque)
{
  march4cpp::IMotionCube imc = march4cpp::IMotionCube(1, encoder);

  imc.setActuationMode(march4cpp::ActuationMode("position"));
  ASSERT_DEATH(imc.actuateTorque(1), "trying to actuate torque, while actuationmode = position");
}

TEST_F(IMotionCubeTest, ChangeActuationModePosition)
{
  march4cpp::IMotionCube imc = march4cpp::IMotionCube(1, encoder);
  ASSERT_EQ(march4cpp::ActuationMode::unknown, imc.getActuationMode().getValue());

  imc.setActuationMode(march4cpp::ActuationMode("position"));
  ASSERT_EQ(march4cpp::ActuationMode::position, imc.getActuationMode().getValue());
}

TEST_F(IMotionCubeTest, ChangeActuationModeTorque)
{
  march4cpp::IMotionCube imc = march4cpp::IMotionCube(1, encoder);
  ASSERT_EQ(march4cpp::ActuationMode::unknown, imc.getActuationMode().getValue());

  imc.setActuationMode(march4cpp::ActuationMode("torque"));
  ASSERT_EQ(march4cpp::ActuationMode::torque, imc.getActuationMode().getValue());
}

TEST_F(IMotionCubeTest, ChangeActuationModeToUnknown)
{
  march4cpp::IMotionCube imc = march4cpp::IMotionCube(1, encoder);
  ASSERT_EQ(march4cpp::ActuationMode::unknown, imc.getActuationMode().getValue());

  imc.setActuationMode(march4cpp::ActuationMode("position"));
  ASSERT_EQ(march4cpp::ActuationMode::position, imc.getActuationMode().getValue());

  ASSERT_THROW(imc.setActuationMode(march4cpp::ActuationMode("unknown")), std::runtime_error);
}

TEST_F(IMotionCubeTest, ChangeActuationModeFromPositionToTorque)
{
  march4cpp::IMotionCube imc = march4cpp::IMotionCube(1, encoder);
  imc.setActuationMode(march4cpp::ActuationMode("position"));
  ASSERT_EQ(march4cpp::ActuationMode::position, imc.getActuationMode().getValue());

  ASSERT_THROW(imc.setActuationMode(march4cpp::ActuationMode("torque")), std::runtime_error);
}

TEST_F(IMotionCubeTest, ChangeActuationModeFromTorqueToPosition)
{
  march4cpp::IMotionCube imc = march4cpp::IMotionCube(1, encoder);
  imc.setActuationMode(march4cpp::ActuationMode("torque"));
  ASSERT_EQ(march4cpp::ActuationMode::torque, imc.getActuationMode().getValue());

  ASSERT_THROW(imc.setActuationMode(march4cpp::ActuationMode("position")), std::runtime_error);
}