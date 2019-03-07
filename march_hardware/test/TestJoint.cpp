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

TEST_F(TestJoint, SlaveIndexZero)
{
}

TEST_F(TestJoint, SlaveIndexMinusOne)
{
}
