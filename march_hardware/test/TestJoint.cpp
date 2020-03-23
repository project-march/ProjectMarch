// Copyright 2018 Project March.

#include "gtest/gtest.h"

#include "march_hardware/error/hardware_exception.h"
#include "march_hardware/Joint.h"
#include "march_hardware/TemperatureGES.h"
#include "march_hardware/IMotionCube.h"
#include "mocks/MockTemperatureGES.cpp"
#include "mocks/MockIMotionCube.cpp"

class JointTest : public ::testing::Test
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

TEST_F(JointTest, ActuatePositionDisableActuation)
{
  march::Joint joint(this->imc);
  joint.setAllowActuation(false);
  joint.setName("actuate_false");
  EXPECT_FALSE(joint.canActuate());
  ASSERT_THROW(joint.actuateRad(0.3), march::error::HardwareException);
}

TEST_F(JointTest, ActuateTorqueDisableActuation)
{
  march::Joint joint(this->imc);
  joint.setAllowActuation(false);
  joint.setName("actuate_false");
  EXPECT_FALSE(joint.canActuate());
  ASSERT_THROW(joint.actuateTorque(3), march::error::HardwareException);
}

TEST_F(JointTest, PrepareForActuationWithUnknownMode)
{
  march::Joint joint(this->imc);
  joint.setAllowActuation(true);
  ASSERT_THROW(joint.prepareActuation(), march::error::HardwareException);
}
