#ifndef __clang_analyzer__
// NOLINTBEGIN
// Copyright 2023 Project March.
#include "march_hardware/motor_controller/motor_controller_type.h"
#include "march_hardware/torque_sensor/torque_sensor.h"

#include <cmath>
#include <gtest/gtest.h>

class TorqueSensorTest : public testing::Test {
protected:
    const size_t max_torque = 10;
    const size_t average_torque = 10;
    const march::MotorControllerType motor_controller_type = march::MotorControllerType::ODrive;
    march::TorqueSensor torque_sensor = march::TorqueSensor(motor_controller_type, max_torque, average_torque);
};

TEST_F(TorqueSensorTest, getMotorControllerTypeTest)
{
    ASSERT_EQ(this->torque_sensor.getMotorControllerType(), this->motor_controller_type);
}

TEST_F(TorqueSensorTest, getMaxTorqueTest)
{
    ASSERT_EQ(this->torque_sensor.getMaxTorque(), this->max_torque);
}

TEST_F(TorqueSensorTest, exceedMaxTorqueTrue)
{
    ASSERT_TRUE(this->torque_sensor.exceedsMaxTorque(12.f));
}

TEST_F(TorqueSensorTest, exceedsMaxTorqueFalse)
{
    ASSERT_FALSE(this->torque_sensor.exceedsMaxTorque(8.f));
}

// NOLINTEND
#endif
