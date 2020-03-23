// Copyright 2018 Project March.
#include "mocks/MockTemperatureGES.cpp"
#include "mocks/MockIMotionCube.cpp"
#include "march_hardware/error/hardware_exception.h"
#include "march_hardware/Joint.h"

#include <memory>
#include <utility>

#include <gtest/gtest.h>
#include <gmock/gmock.h>

using testing::Eq;
using testing::Return;

class JointTest : public testing::Test
{
protected:
  void SetUp() override
  {
    this->imc = std::make_unique<MockIMotionCube>();
    this->temperature_ges = std::make_unique<MockTemperatureGES>();
  }

  std::unique_ptr<MockIMotionCube> imc;
  std::unique_ptr<MockTemperatureGES> temperature_ges;
};

TEST_F(JointTest, InitializeWithoutMotorControllerAndGes)
{
  march::Joint joint("test", 0);
  ASSERT_NO_THROW(joint.initialize(1));
}

TEST_F(JointTest, InitializeWithoutTemperatureGes)
{
  const int expected_cycle = 3;
  EXPECT_CALL(*this->imc, writeInitialSDOs(Eq(expected_cycle))).Times(1);

  march::Joint joint("test", 0, false, std::move(this->imc));
  ASSERT_NO_THROW(joint.initialize(expected_cycle));
}

TEST_F(JointTest, InitializeWithoutMotorController)
{
  const int expected_cycle = 3;
  EXPECT_CALL(*this->temperature_ges, writeInitialSDOs(Eq(expected_cycle))).Times(1);

  march::Joint joint("test", 0, false, nullptr, std::move(this->temperature_ges));
  ASSERT_NO_THROW(joint.initialize(expected_cycle));
}

TEST_F(JointTest, AllowActuation)
{
  march::Joint joint("test", 0, true, std::move(this->imc));
  ASSERT_TRUE(joint.canActuate());
}

TEST_F(JointTest, DisallowActuationWithoutMotorController)
{
  march::Joint joint("test", 0, true, nullptr);
  ASSERT_FALSE(joint.canActuate());
}

TEST_F(JointTest, DisableActuation)
{
  march::Joint joint("test", 0, false, std::move(this->imc));
  ASSERT_FALSE(joint.canActuate());
}

TEST_F(JointTest, ActuatePositionDisableActuation)
{
  march::Joint joint("actuate_false", 0, false, std::move(this->imc));
  EXPECT_FALSE(joint.canActuate());
  ASSERT_THROW(joint.actuateRad(0.3), march::error::HardwareException);
}

TEST_F(JointTest, ActuatePosition)
{
  const double expected_rad = 5;
  EXPECT_CALL(*this->imc, actuateRad(Eq(expected_rad))).Times(1);

  march::Joint joint("actuate_false", 0, true, std::move(this->imc));
  ASSERT_NO_THROW(joint.actuateRad(expected_rad));
}

TEST_F(JointTest, ActuateTorqueDisableActuation)
{
  march::Joint joint("actuate_false", 0, false, std::move(this->imc));
  EXPECT_FALSE(joint.canActuate());
  ASSERT_THROW(joint.actuateTorque(3), march::error::HardwareException);
}

TEST_F(JointTest, ActuateTorque)
{
  const int16_t expected_torque = 5;
  EXPECT_CALL(*this->imc, actuateTorque(Eq(expected_torque))).Times(1);

  march::Joint joint("actuate_false", 0, true, std::move(this->imc));
  ASSERT_NO_THROW(joint.actuateTorque(expected_torque));
}

TEST_F(JointTest, PrepareForActuationNotAllowed)
{
  march::Joint joint("actuate_false", 0, false, std::move(this->imc));
  ASSERT_THROW(joint.prepareActuation(), march::error::HardwareException);
}

TEST_F(JointTest, PrepareForActuationAllowed)
{
  EXPECT_CALL(*this->imc, goToOperationEnabled()).Times(1);
  march::Joint joint("actuate_true", 0, true, std::move(this->imc));
  ASSERT_NO_THROW(joint.prepareActuation());
}

TEST_F(JointTest, GetTemperature)
{
  const float expected_temperature = 45.0;
  EXPECT_CALL(*this->temperature_ges, getTemperature()).WillOnce(Return(expected_temperature));

  march::Joint joint("get_temperature", 0, false, nullptr, std::move(this->temperature_ges));
  ASSERT_FLOAT_EQ(joint.getTemperature(), expected_temperature);
}

TEST_F(JointTest, GetTemperatureWithoutTemperatureGes)
{
  march::Joint joint("get_temperature", 0, false, nullptr, nullptr);
  ASSERT_FLOAT_EQ(joint.getTemperature(), -1.0);
}
