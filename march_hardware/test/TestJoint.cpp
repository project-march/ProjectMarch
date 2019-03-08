// Copyright 2018 Project March.

#include "gtest/gtest.h"
#include <gmock/gmock.h>

#include "march_hardware/Joint.h"
#include "march_hardware/TemperatureGES.h"
#include "march_hardware/IMotionCube.h"
#include "mocks/MockTemperatureGES.cpp"
#include "mocks/MockIMotionCube.cpp"

using ::testing::Return;
using ::testing::AtLeast;
using ::testing::AtMost;

class TestJoint : public ::testing::Test
{
protected:
  const float temperature = 42;
};

// TODO(Tim) write joint tests (with mocking)