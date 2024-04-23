#ifndef __clang_analyzer__
// NOLINTBEGIN
// Copyright 2018 Project March.
#include "march_hardware/error/hardware_exception.h"
#include "march_hardware/joint.h"
#include "march_hardware/motor_controller/odrive/odrive_state.h"
#include "mocks/mock_motorcontroller_state.h"
#include "mocks/mock_odrive.h"
#include "mocks/mock_temperature_ges.h"

#include <memory>
#include <utility>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

using testing::_;
using testing::ByMove;
using testing::Eq;
using testing::Return;

class JointTest : public testing::Test {
protected:
    void SetUp() override
    {
        this->logger_ = std::make_shared<march_logger::RosLogger>("march_hardware_builder");
        this->odrive = std::make_unique<MockODrive>();
        this->temperature_ges = std::make_unique<MockTemperatureGES>();
    }
    std::shared_ptr<march_logger::BaseLogger> logger_;
    std::unique_ptr<MockODrive> odrive;
    std::unique_ptr<MockTemperatureGES> temperature_ges;
};

TEST_F(JointTest, InitializeWithoutTemperatureGes)
{
    const int expected_cycle = 3;
    EXPECT_CALL(*this->odrive, initSdo(_, Eq(expected_cycle))).Times(/*n=*/1);

    march::Joint joint("test", /*net_number=*/0, std::move(this->odrive), this->logger_);
    ASSERT_NO_THROW(joint.initSdo(expected_cycle));
}

TEST_F(JointTest, GetName)
{
    const std::string expected_name = "test";
    march::Joint joint(expected_name, /*net_number=*/0, std::move(this->odrive), this->logger_);
    ASSERT_EQ(expected_name, joint.getName());
}

TEST_F(JointTest, GetNetNumber)
{
    const int expected_net_number = 2;
    march::Joint joint("test", expected_net_number, std::move(this->odrive), this->logger_);
    ASSERT_EQ(expected_net_number, joint.getNetNumber());
}

TEST_F(JointTest, ActuatePosition)
{
    const double expected_rad = 5;
    EXPECT_CALL(*this->odrive, actuateRadians(Eq(expected_rad))).Times(/*n=*/1);

    march::Joint joint("actuate_false", /*net_number=*/0, std::move(this->odrive), this->logger_);
    joint.getMotorController()->setActuationMode(march::ActuationMode::position);
    ASSERT_NO_THROW(joint.actuate(expected_rad));
}

TEST_F(JointTest, ActuateTorque)
{
    const double expected_torque = 5;
    EXPECT_CALL(*this->odrive, actuateTorque(Eq(expected_torque))).Times(/*n=*/1);

    march::Joint joint("actuate_true", /*net_number=*/0, std::move(this->odrive), this->logger_);
    joint.getMotorController()->setActuationMode(march::ActuationMode::torque);

    ASSERT_NO_THROW(joint.actuate(expected_torque));
}

TEST_F(JointTest, PrepareForActuationAllowed)
{
    EXPECT_CALL(*this->odrive, prepareActuation()).Times(/*n=*/1);
    march::Joint joint("actuate_true", /*net_number=*/0, std::move(this->odrive), this->logger_);
    ASSERT_NO_THROW(joint.prepareActuation());
}

TEST_F(JointTest, hasTemperatureGes)
{
    march::Joint joint("get_temperature", /*net_number=*/0, std::move(this->odrive), this->logger_);
    ASSERT_FALSE(joint.hasTemperatureGES());
}

TEST_F(JointTest, ResetController)
{
    EXPECT_CALL(*this->odrive, resetSlave(_)).Times(/*n=*/1);
    march::Joint joint("reset_controller", /*net_number=*/0, std::move(this->odrive), this->logger_);
    ASSERT_NO_THROW(joint.getMotorController()->resetSlave());
}

TEST_F(JointTest, TestPrepareActuation)
{
    EXPECT_CALL(*this->odrive, prepareActuation()).Times(/*n=*/1);
    march::Joint joint("actuate_true", /*net_number=*/0, std::move(this->odrive), this->logger_);
    joint.prepareActuation();
}

TEST_F(JointTest, TestReceivedDataUpdateFirstTimeTrue)
{
    EXPECT_CALL(*this->odrive, getState()).Times(/*n=*/1);
    march::Joint joint("actuate_true", /*net_number=*/0, std::move(this->odrive), this->logger_);
    ASSERT_TRUE(joint.receivedDataUpdate());
}

TEST_F(JointTest, TestReceivedDataUpdateTrue)
{
    auto basic_state = std::make_unique<MockMotorControllerState>();
    auto new_state = std::make_unique<MockMotorControllerState>();
    new_state->motor_current_ = 11;
    EXPECT_CALL(*this->odrive, getState())
        .WillOnce(Return(ByMove(std::move(basic_state))))
        .WillOnce(Return(ByMove(std::move(new_state))));
    march::Joint joint("actuate_true", /*net_number=*/0, std::move(this->odrive), this->logger_);
    ASSERT_TRUE(joint.receivedDataUpdate());
    ASSERT_TRUE(joint.receivedDataUpdate());
}

// TEST_F(JointTest, TestReceivedDataUpdateFalse)
//{
//    auto basic_state = std::make_unique<MockMotorControllerState>();
//    auto new_state = std::make_unique<MockMotorControllerState>();
//    EXPECT_CALL(*this->odrive, getState())
//        .WillOnce(Return(ByMove(std::move(basic_state))))
//        .WillOnce(Return(ByMove(std::move(new_state))));
//    march::Joint joint("actuate_true", /*net_number=*/0, std::move(this->odrive), this->logger_);
//    ASSERT_TRUE(joint.receivedDataUpdate());
//    ASSERT_FALSE(joint.receivedDataUpdate());
//}

// TEST_F(JointTest, TestReadEncodersOnce)
//{
//    double velocity = 0.5;
//    double initial_absolute_position = 3;
//    double initial_incremental_position = 5;
//    double new_incremental_position = 4;
//
//    EXPECT_CALL(*this->odrive, isIncrementalEncoderMorePrecise()).WillRepeatedly(Return(/*value=*/true));
//    EXPECT_CALL(*this->odrive, getState()).Times(/*n=*/1);
//    EXPECT_CALL(*this->odrive, getIncrementalPositionUnchecked())
//        .WillOnce(Return(/*value=*/initial_incremental_position))
//        .WillOnce(Return(/*value=*/new_incremental_position));
//    EXPECT_CALL(*this->odrive, getAbsolutePositionUnchecked()).WillOnce(Return(/*value=*/initial_absolute_position));
//
//    EXPECT_CALL(*this->odrive, getIncrementalVelocityUnchecked()).WillOnce(Return(/*value=*/velocity));
//
//    march::Joint joint("actuate_true", /*net_number=*/0, std::move(this->odrive), this->logger_);
//    joint.prepareActuation();
//    joint.readFirstEncoderValues(/*operational_check=*/false);
//    joint.readEncoders();
//
//    double expected_position = initial_absolute_position + (new_incremental_position - initial_incremental_position);
//
//    ASSERT_DOUBLE_EQ(joint.getPosition(), expected_position);
//    ASSERT_DOUBLE_EQ(joint.getVelocity(), velocity);
//}

// TEST_F(JointTest, TestReadEncodersTwice)
//{
//    double first_velocity = 10;
//    double second_velocity = 8;
//    double initial_absolute_position = 2;
//    double first_incremental_position = 5;
//    double second_incremental_position = 4;
//    double third_incremental_position = 6;
//
//    // receivedDataUpdate should return true
//    auto basic_state = std::make_unique<MockMotorControllerState>();
//    auto new_state = std::make_unique<MockMotorControllerState>();
//    new_state->motor_current_ = 11;
//    EXPECT_CALL(*this->odrive, getState())
//        .WillOnce(Return(ByMove(std::move(basic_state))))
//        .WillOnce(Return(ByMove(std::move(new_state))));
//
//    EXPECT_CALL(*this->odrive, isIncrementalEncoderMorePrecise()).WillRepeatedly(Return(/*value=*/true));
//
//    EXPECT_CALL(*this->odrive, getIncrementalPositionUnchecked())
//        .WillOnce(Return(first_incremental_position))
//        .WillOnce(Return(second_incremental_position))
//        .WillOnce(Return(third_incremental_position));
//    EXPECT_CALL(*this->odrive, getAbsolutePositionUnchecked()).WillOnce(Return(initial_absolute_position));
//
//    EXPECT_CALL(*this->odrive, getIncrementalVelocityUnchecked())
//        .WillOnce(Return(first_velocity))
//        .WillOnce(Return(second_velocity));
//
//    march::Joint joint("actuate_true", /*net_number=*/0, std::move(this->odrive), this->logger_);
//    joint.prepareActuation();
//    joint.readFirstEncoderValues(/*operational_check=*/false);
//    joint.readEncoders();
//    joint.readEncoders();
//
//    double expected_position = initial_absolute_position + (third_incremental_position - second_incremental_position)
//        + (second_incremental_position - first_incremental_position);
//
//    ASSERT_DOUBLE_EQ(joint.getPosition(), expected_position);
//    ASSERT_DOUBLE_EQ(joint.getVelocity(), second_velocity);
//}

// TEST_F(JointTest, TestReadEncodersNoUpdate)
//{
//    double velocity = 0.5;
//    double first_incremental_position = 5;
//    double second_incremental_position = 4;
//    double initial_absolute_position = 3;
//
//    // receivedDataUpdate should return false
//    auto basic_state = std::make_unique<MockMotorControllerState>();
//    auto new_state = std::make_unique<MockMotorControllerState>();
//    EXPECT_CALL(*this->odrive, getState())
//        .WillOnce(Return(ByMove(std::move(basic_state))))
//        .WillOnce(Return(ByMove(std::move(new_state))));
//
//    EXPECT_CALL(*this->odrive, getIncrementalPositionUnchecked())
//        .WillOnce(Return(first_incremental_position))
//        .WillOnce(Return(second_incremental_position));
//    EXPECT_CALL(*this->odrive, getAbsolutePositionUnchecked()).WillOnce(Return(initial_absolute_position));
//    EXPECT_CALL(*this->odrive, getIncrementalVelocityUnchecked()).WillOnce(Return(velocity));
//
//    march::Joint joint("actuate_true", /*net_number=*/0, std::move(this->odrive), this->logger_);
//    joint.prepareActuation();
//    joint.readFirstEncoderValues(/*operational_check=*/false);
//    joint.readEncoders();
//    joint.readEncoders();
//
//    double expected_position = initial_absolute_position + (second_incremental_position - first_incremental_position);
//
//    ASSERT_DOUBLE_EQ(joint.getPosition(), expected_position);
//    ASSERT_DOUBLE_EQ(joint.getVelocity(), velocity);
//}
// NOLINTEND
#endif
