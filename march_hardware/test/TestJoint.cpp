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

TEST_F(JointTest, SetAllowActuation)
{
  march::Joint joint("test", 0, false, std::move(this->imc));
  ASSERT_FALSE(joint.canActuate());
  joint.setAllowActuation(true);
  ASSERT_TRUE(joint.canActuate());
}

TEST_F(JointTest, GetName)
{
  const std::string expected_name = "test";
  march::Joint joint(expected_name, 0, false, std::move(this->imc));
  ASSERT_EQ(expected_name, joint.getName());
}

TEST_F(JointTest, GetNetNumber)
{
  const int expected_net_number = 2;
  march::Joint joint("test", expected_net_number, false, std::move(this->imc));
  ASSERT_EQ(expected_net_number, joint.getNetNumber());
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

TEST_F(JointTest, ResetController)
{
  EXPECT_CALL(*this->imc, reset()).Times(1);
  march::Joint joint("reset_controller", 0, true, std::move(this->imc));
  ASSERT_NO_THROW(joint.resetIMotionCube());
}

TEST_F(JointTest, ResetControllerWithoutController)
{
  EXPECT_CALL(*this->imc, reset()).Times(0);
  march::Joint joint("reset_controller", 0, true, nullptr, std::move(this->temperature_ges));
  ASSERT_NO_THROW(joint.resetIMotionCube());
}

TEST_F(JointTest, TestPrepareActuation)
{
  EXPECT_CALL(*this->imc, getAngleRadIncremental()).WillOnce(Return(5));
  EXPECT_CALL(*this->imc, getAngleRadAbsolute()).WillOnce(Return(3));
  EXPECT_CALL(*this->imc, goToOperationEnabled()).Times(1);
  march::Joint joint("actuate_true", 0, true, std::move(this->imc));
  joint.prepareActuation();
  ASSERT_EQ(joint.getIncrementalPosition(), 5);
  ASSERT_EQ(joint.getAbsolutePosition(), 3);
}

TEST_F(JointTest, TestReceivedDataUpdateFirstTimeTrue)
{
  EXPECT_CALL(*this->imc, getIMCVoltage()).WillOnce(Return(48));
  march::Joint joint("actuate_true", 0, true, std::move(this->imc));
  ASSERT_TRUE(joint.receivedDataUpdate());
}

TEST_F(JointTest, TestReceivedDataUpdateTrue)
{
  EXPECT_CALL(*this->imc, getIMCVoltage()).WillOnce(Return(48)).WillOnce(Return(48.001));
  march::Joint joint("actuate_true", 0, true, std::move(this->imc));
  joint.receivedDataUpdate();
  ASSERT_TRUE(joint.receivedDataUpdate());
}

TEST_F(JointTest, TestReceivedDataUpdateFalse)
{
  EXPECT_CALL(*this->imc, getIMCVoltage()).WillRepeatedly(Return(48));
  march::Joint joint("actuate_true", 0, true, std::move(this->imc));
  joint.receivedDataUpdate();
  ASSERT_FALSE(joint.receivedDataUpdate());
}

TEST_F(JointTest, TestReadEncodersOnce)
{
  ros::Duration elapsed_time(0.2);
  double velocity = 0.5;
  double absolute_noise = -2 * this->imc->getAbsoluteRadPerBit();

  double initial_incremental_position = 5;
  double initial_absolute_position = 3;
  double new_incremental_position = initial_incremental_position + velocity * elapsed_time.toSec();
  double new_absolute_position = initial_absolute_position + velocity * elapsed_time.toSec() + absolute_noise;

  EXPECT_CALL(*this->imc, getIMCVoltage()).WillOnce(Return(48));
  EXPECT_CALL(*this->imc, getAngleRadIncremental())
      .WillOnce(Return(initial_incremental_position))
      .WillOnce(Return(new_incremental_position));
  EXPECT_CALL(*this->imc, getAngleRadAbsolute())
      .WillOnce(Return(initial_absolute_position))
      .WillOnce(Return(new_absolute_position));

  march::Joint joint("actuate_true", 0, true, std::move(this->imc));
  joint.prepareActuation();

  joint.readEncoders(elapsed_time);

  //  ASSERT_DOUBLE_EQ(joint.getPosition(), initial_absolute_position + velocity * elapsed_time.toSec());
  ASSERT_DOUBLE_EQ(joint.getVelocity(),
                   (new_incremental_position - initial_incremental_position) / elapsed_time.toSec());
}

TEST_F(JointTest, TestReadEncodersTwice)
{
  ros::Duration elapsed_time(0.2);
  double first_velocity = 0.5;
  double second_velocity = 0.8;

  double absolute_noise = -this->imc->getAbsoluteRadPerBit();

  double initial_incremental_position = 5;
  double initial_absolute_position = 3;
  double second_incremental_position = initial_incremental_position + first_velocity * elapsed_time.toSec();
  double second_absolute_position = initial_absolute_position + first_velocity * elapsed_time.toSec() + absolute_noise;
  double third_incremental_position = second_incremental_position + second_velocity * elapsed_time.toSec();
  double third_absolute_position = second_absolute_position + second_velocity * elapsed_time.toSec() + absolute_noise;

  EXPECT_CALL(*this->imc, getIMCVoltage()).WillOnce(Return(48)).WillOnce(Return(48.01));
  EXPECT_CALL(*this->imc, getAngleRadIncremental())
      .WillOnce(Return(initial_incremental_position))
      .WillOnce(Return(second_incremental_position))
      .WillOnce(Return(third_incremental_position));
  EXPECT_CALL(*this->imc, getAngleRadAbsolute())
      .WillOnce(Return(initial_absolute_position))
      .WillOnce(Return(second_absolute_position))
      .WillOnce(Return(third_absolute_position));

  march::Joint joint("actuate_true", 0, true, std::move(this->imc));
  joint.prepareActuation();

  joint.readEncoders(elapsed_time);
  joint.readEncoders(elapsed_time);

  ASSERT_DOUBLE_EQ(joint.getPosition(),
                   initial_absolute_position + (first_velocity + second_velocity) * elapsed_time.toSec());
  ASSERT_DOUBLE_EQ(joint.getVelocity(),
                   (third_incremental_position - second_incremental_position) / elapsed_time.toSec());
}
