#ifndef __clang_analyzer__
// NOLINTBEGIN
// Copyright 2020 Project March.
#include "march_hardware/encoder/incremental_encoder.h"
#include "march_hardware/error/hardware_exception.h"
#include "march_hardware/motor_controller/motor_controller_type.h"

#include <cmath>

#include <gtest/gtest.h>

class IncrementalEncoderTest : public testing::Test {
protected:
    const size_t counts_per_rotation = 1 << 12;
    const double transmission = 100;
    const march::MotorControllerType motor_controller_type = march::MotorControllerType::ODrive;
    march::IncrementalEncoder encoder
        = march::IncrementalEncoder(this->counts_per_rotation, this->motor_controller_type, this->transmission);
};

TEST_F(IncrementalEncoderTest, ZeroIUToRad)
{
    ASSERT_EQ(0.0, this->encoder.positionIUToRadians(0));
}

TEST_F(IncrementalEncoderTest, GetTransmission)
{
    ASSERT_DOUBLE_EQ(this->transmission, this->encoder.getTransmission());
}

TEST_F(IncrementalEncoderTest, CorrectToRad)
{
    const int32_t iu = 1000;
    const double expected = iu * 2.0 * M_PI / (this->counts_per_rotation * this->transmission);
    ASSERT_DOUBLE_EQ(expected, this->encoder.positionIUToRadians(iu));
}

TEST_F(IncrementalEncoderTest, VecolityIUToRadians)
{
    const double velocity = 10.0;
    const double expected = 0.62831853071795862;
    ASSERT_DOUBLE_EQ(expected, this->encoder.velocityIUToRadians(velocity));
}

TEST_F(IncrementalEncoderTest, velocityRadiansToIU)
{
    const double expected = 10;
    const double velocity = 0.62831853071795862;
    ASSERT_DOUBLE_EQ(expected, this->encoder.velocityRadiansToIU(velocity));
}

TEST_F(IncrementalEncoderTest, InvalidMotorControllerType)
{
    march::IncrementalEncoder bad_encoder = march::IncrementalEncoder(
        this->counts_per_rotation, march::MotorControllerType::IMotionCube, this->transmission);
    const double velocity = 0.62831853071795862;
    ASSERT_THROW(bad_encoder.velocityRadiansToIU(velocity), march::error::HardwareException);
    ASSERT_THROW(bad_encoder.velocityIUToRadians(velocity), march::error::HardwareException);
}
// NOLINTEND
#endif
