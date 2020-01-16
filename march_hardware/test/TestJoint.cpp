// Copyright 2018 Project March.

#include "gtest/gtest.h"
#include <gmock/gmock.h>

#include "march_hardware/Joint.h"
#include "march_hardware/TemperatureGES.h"
#include "march_hardware/IMotionCube.h"
#include "mocks/MockTemperatureGES.cpp"
#include "mocks/MockIMotionCube.cpp"

using ::testing::AtLeast;
using ::testing::AtMost;
using ::testing::Return;

class JointTest : public ::testing::Test
{
protected:
  const float temperature = 42;
  const MockIMotionCube imc;
};

class JointDeathTest : public ::testing::Test
{
protected:
  const float temperature = 42;
  const MockIMotionCube imc;
};

TEST_F(JointTest, AllowActuation)
{
  march::Joint joint(this->imc);
  joint.setAllowActuation(true);
  ASSERT_TRUE(joint.canActuate());
}

TEST_F(JointTest, DisableActuation)
{
  march::Joint joint(this->imc);
  joint.setAllowActuation(false);
  ASSERT_FALSE(joint.canActuate());
}

TEST_F(JointDeathTest, ActuateDisableActuation)
{
  march::Joint joint(this->imc);
  joint.setAllowActuation(false);
  joint.setName("actuate_false");
  ASSERT_FALSE(joint.canActuate());
  ASSERT_DEATH(joint.actuateRad(0.3), "Joint actuate_false is not allowed to actuate, "
                                      "yet its actuate method has been called");
}

TEST_F(JointTest, NoActuationMode)
{
  march::Joint joint(this->imc);
  joint.setName("test_joint");
  ASSERT_DEATH(joint.actuateRad(1), "Joint test_joint is not allowed to actuate, "
                                    "yet its actuate method has been called");
}
