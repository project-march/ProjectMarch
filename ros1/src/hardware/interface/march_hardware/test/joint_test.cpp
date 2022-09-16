// Copyright 2018 Project March.
#include "march_hardware/error/hardware_exception.h"
#include "march_hardware/joint.h"
#include "march_hardware/motor_controller/imotioncube/imotioncube_state.h"
#include "mocks/mock_imotioncube.h"
#include "mocks/mock_motorcontroller_state.h"
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
        this->imc = std::make_unique<MockIMotionCube>();
        this->temperature_ges = std::make_unique<MockTemperatureGES>();
    }

    std::unique_ptr<MockIMotionCube> imc;
    std::unique_ptr<MockTemperatureGES> temperature_ges;
};

TEST_F(JointTest, InitializeWithoutTemperatureGes)
{
    const int expected_cycle = 3;
    EXPECT_CALL(*this->imc, initSdo(_, Eq(expected_cycle))).Times(/*n=*/1);

    march::Joint joint("test", /*net_number=*/0, std::move(this->imc));
    ASSERT_NO_THROW(joint.initSdo(expected_cycle));
}

TEST_F(JointTest, GetName)
{
    const std::string expected_name = "test";
    march::Joint joint(expected_name, /*net_number=*/0, std::move(this->imc));
    ASSERT_EQ(expected_name, joint.getName());
}

TEST_F(JointTest, GetNetNumber)
{
    const int expected_net_number = 2;
    march::Joint joint("test", expected_net_number, std::move(this->imc));
    ASSERT_EQ(expected_net_number, joint.getNetNumber());
}

TEST_F(JointTest, ActuatePosition)
{
    const double expected_rad = 5;
    EXPECT_CALL(*this->imc, actuateRadians(Eq(expected_rad))).Times(/*n=*/1);

    march::Joint joint("actuate_false", /*net_number=*/0, std::move(this->imc));
    joint.getMotorController()->setActuationMode(march::ActuationMode::position);
    ASSERT_NO_THROW(joint.actuate(expected_rad));
}

TEST_F(JointTest, ActuateTorque)
{
    const double expected_torque = 5;
    EXPECT_CALL(*this->imc, actuateTorque(Eq(expected_torque))).Times(/*n=*/1);

    march::Joint joint("actuate_true", /*net_number=*/0, std::move(this->imc));
    joint.getMotorController()->setActuationMode(march::ActuationMode::torque);

    ASSERT_NO_THROW(joint.actuate(expected_torque));
}

TEST_F(JointTest, PrepareForActuationAllowed)
{
    EXPECT_CALL(*this->imc, prepareActuation()).Times(/*n=*/1);
    march::Joint joint("actuate_true", /*net_number=*/0, std::move(this->imc));
    ASSERT_NO_THROW(joint.prepareActuation());
}

TEST_F(JointTest, hasTemperatureGes)
{
    march::Joint joint("get_temperature", /*net_number=*/0, std::move(this->imc));
    ASSERT_FALSE(joint.hasTemperatureGES());
}

TEST_F(JointTest, ResetController)
{
    EXPECT_CALL(*this->imc, resetSlave(_)).Times(/*n=*/1);
    march::Joint joint("reset_controller", /*net_number=*/0, std::move(this->imc));
    ASSERT_NO_THROW(joint.getMotorController()->resetSlave());
}

TEST_F(JointTest, TestPrepareActuation)
{
    EXPECT_CALL(*this->imc, prepareActuation()).Times(/*n=*/1);
    march::Joint joint("actuate_true", /*net_number=*/0, std::move(this->imc));
    joint.prepareActuation();
}

TEST_F(JointTest, TestReceivedDataUpdateFirstTimeTrue)
{
    EXPECT_CALL(*this->imc, getState()).Times(/*n=*/1);
    march::Joint joint("actuate_true", /*net_number=*/0, std::move(this->imc));
    ASSERT_TRUE(joint.receivedDataUpdate());
}

TEST_F(JointTest, TestReceivedDataUpdateTrue)
{
    auto basic_state = std::make_unique<MockMotorControllerState>();
    auto new_state = std::make_unique<MockMotorControllerState>();
    new_state->motor_current_ = 11;
    EXPECT_CALL(*this->imc, getState())
        .WillOnce(Return(ByMove(std::move(basic_state))))
        .WillOnce(Return(ByMove(std::move(new_state))));
    march::Joint joint("actuate_true", /*net_number=*/0, std::move(this->imc));
    ASSERT_TRUE(joint.receivedDataUpdate());
    ASSERT_TRUE(joint.receivedDataUpdate());
}

TEST_F(JointTest, TestReceivedDataUpdateFalse)
{
    auto basic_state = std::make_unique<MockMotorControllerState>();
    auto new_state = std::make_unique<MockMotorControllerState>();
    EXPECT_CALL(*this->imc, getState())
        .WillOnce(Return(ByMove(std::move(basic_state))))
        .WillOnce(Return(ByMove(std::move(new_state))));
    march::Joint joint("actuate_true", /*net_number=*/0, std::move(this->imc));
    ASSERT_TRUE(joint.receivedDataUpdate());
    ASSERT_FALSE(joint.receivedDataUpdate());
}
