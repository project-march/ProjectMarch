// NOLINTBEGIN
// Copyright 2020 Project March.
#include "march_hardware/encoder/incremental_encoder.h"
#include "march_hardware/motor_controller/motor_controller_type.h"

#include <cmath>

#include <gtest/gtest.h>

class IncrementalEncoderTest : public testing::Test {
protected:
    const size_t counts_per_rotation = 1 << 12;
    const double transmission = 100;
    const march::MotorControllerType motor_controller_type = march::MotorControllerType::IMotionCube;
    march::IncrementalEncoder encoder
        = march::IncrementalEncoder(this->counts_per_rotation, this->motor_controller_type, this->transmission);
};

TEST_F(IncrementalEncoderTest, ZeroIUToRad)
{
    ASSERT_EQ(0.0, this->encoder.positionIUToRadians(0));
}

TEST_F(IncrementalEncoderTest, CorrectToRad)
{
    const int32_t iu = 1000;
    const double expected = iu * 2.0 * M_PI / (this->counts_per_rotation * this->transmission);
    ASSERT_DOUBLE_EQ(expected, this->encoder.positionIUToRadians(iu));
}
// NOLINTEND
