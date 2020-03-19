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
  MockIMotionCube imc;
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

TEST_F(JointTest, ActuateDisableActuation)
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

TEST_F(JointTest, TestPrepareActuation)
{
  EXPECT_CALL(this->imc, getAngleRadIncremental()).WillOnce(Return(5));
  EXPECT_CALL(this->imc, getAngleRadAbsolute()).WillOnce(Return(0.3));
  march::Joint joint(this->imc);
  joint.setAllowActuation(true);
  std::cerr << "[          ] BAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA = " << std::endl;
  joint->iMotionCube.goToOperationEnabled();
  joint.prepareActuation();
  ASSERT_EQ(joint.getIncremental_position(), 5);
  ASSERT_EQ(joint.getAbsolute_position(), 0.3);
}
