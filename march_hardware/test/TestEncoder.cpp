// Copyright 2018 Project March.

#include "gtest/gtest.h"
#include <gmock/gmock.h>

#include "march_hardware/Joint.h"
#include "march_hardware/TemperatureGES.h"
#include "march_hardware/IMotionCube.h"
#include "mocks/MockEncoder.cpp"

using ::testing::Return;
using ::testing::AtLeast;
using ::testing::AtMost;

class TestJoint : public ::testing::Test
{
protected:
  // TODO(Tim) make parameterized
  const float angle = 42;
};

TEST_F(TestJoint, Encoder)
{
  MockEncoder mockEncoder;

  EXPECT_CALL(mockEncoder, getAngleDeg()).Times(AtLeast(1));
  ON_CALL(mockEncoder, getAngleDeg()).WillByDefault(Return(angle));

  ASSERT_EQ(angle, mockEncoder.getAngleDeg());
}
