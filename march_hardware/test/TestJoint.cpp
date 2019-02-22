// Copyright 2018 Project March.

#include "gtest/gtest.h"
#include <gmock/gmock.h>

#include "march_hardware/Joint.h"
#include "march_hardware/TemperatureGES.h"
#include "march_hardware/IMotionCube.h"
#include "mocks/MockTemperatureGES.cpp"

using ::testing::Return;
using ::testing::AtLeast;
using ::testing::AtMost;

class TestJoint : public ::testing::Test
{
protected:
  // TODO(Tim) make parameterized
  const float temperature = 42;
};

TEST_F(TestJoint, TemperatureJoint)
{
  MockTemperatureGES mockTemperatureGES;
  march4cpp::Joint emptyJoint = march4cpp::Joint("empty-joint", &mockTemperatureGES);

  EXPECT_CALL(mockTemperatureGES, getTemperature()).Times(AtLeast(1));
  ON_CALL(mockTemperatureGES, getTemperature()).WillByDefault(Return(temperature));

  EXPECT_CALL(mockTemperatureGES, getSlaveIndex()).Times(AtLeast(1));
  ON_CALL(mockTemperatureGES, getSlaveIndex()).WillByDefault(Return(1));

  ASSERT_EQ(temperature, emptyJoint.getTemperature());
}

TEST_F(TestJoint, TemperatureJointIncorrectSlave)
{
  MockTemperatureGES mockTemperatureGES;
  march4cpp::Joint emptyJoint = march4cpp::Joint("empty-joint", &mockTemperatureGES);

  EXPECT_CALL(mockTemperatureGES, getTemperature()).Times(AtMost(0));
  ON_CALL(mockTemperatureGES, getTemperature()).WillByDefault(Return(temperature));

  EXPECT_CALL(mockTemperatureGES, getSlaveIndex()).Times(AtLeast(1));
  ON_CALL(mockTemperatureGES, getSlaveIndex()).WillByDefault(Return(-1));

  ASSERT_EQ(-1, emptyJoint.getTemperature());
}

TEST_F(TestJoint, SlaveIndexZero)
{
}

TEST_F(TestJoint, SlaveIndexMinusOne)
{
}
