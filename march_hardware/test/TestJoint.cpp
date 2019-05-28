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

class JointTest : public ::testing::Test
{
protected:
  const float temperature = 42;
};

class JointDeathTest : public ::testing::Test
{
protected:
    const float temperature = 42;
};

TEST_F(JointTest, AllowActuation)
{
    march4cpp::TemperatureGES temp = march4cpp::TemperatureGES(2,2);
    march4cpp::Joint joint = march4cpp::Joint("actuate_true", true, temp);
    ASSERT_TRUE(joint.canActuate());
}

TEST_F(JointTest, DisableActuation)
{
    march4cpp::TemperatureGES temp = march4cpp::TemperatureGES(2,2);
    march4cpp::Joint joint = march4cpp::Joint("actuate_false", false, temp);
    ASSERT_FALSE(joint.canActuate());
}

TEST_F(JointDeathTest, ActuateDisableActuation)
{
    march4cpp::TemperatureGES temp = march4cpp::TemperatureGES(2,2);
    march4cpp::Joint joint = march4cpp::Joint("actuate_false", false, temp);
    ASSERT_FALSE(joint.canActuate());
    ASSERT_DEATH(joint.actuateRad(0.3), "Joint actuate_false is not allowed to actuate, yet its actuate method has been called.");
}