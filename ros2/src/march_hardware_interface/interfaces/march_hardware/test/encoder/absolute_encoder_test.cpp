#ifndef __clang_analyzer__
// NOLINTBEGIN
// Copyright 2020 Project March.
#include "march_hardware/encoder/absolute_encoder.h"
#include "march_hardware/error/hardware_exception.h"
#include "march_hardware/motor_controller/motor_controller_type.h"

#include <cmath>
#include <tuple>

#include <gtest/gtest.h>

class AbsoluteEncoderTest : public testing::Test {
protected:
    // Values are retrieved from right_knee in march7.yaml
    const size_t counts_per_rotation = 1 << 20;
    const march::MotorControllerType motor_controller_type = march::MotorControllerType::ODrive;
    const int32_t lower_limit_iu = 531696;
    const int32_t upper_limit_iu = 882560;
    const int32_t zero_position_iu = 548304;
    const double lower_soft_limit_rad_diff = 0.0349066;
    const double upper_soft_limit_rad_diff = 0.0349066;

    march::AbsoluteEncoder encoder = march::AbsoluteEncoder(counts_per_rotation, motor_controller_type, lower_limit_iu,
        upper_limit_iu, zero_position_iu, lower_soft_limit_rad_diff, upper_soft_limit_rad_diff);

    const int32_t lower_soft_limit = lower_limit_iu + lower_soft_limit_rad_diff / encoder.getRadiansPerIU();
    const int32_t upper_soft_limit = upper_limit_iu - upper_soft_limit_rad_diff / encoder.getRadiansPerIU();
};

TEST_F(AbsoluteEncoderTest, CorrectLowerHardLimits)
{
    ASSERT_EQ(this->encoder.getLowerHardLimitIU(), this->lower_limit_iu);
}

TEST_F(AbsoluteEncoderTest, CorrectUpperHardLimit)
{
    ASSERT_EQ(this->encoder.getUpperHardLimitIU(), this->upper_limit_iu);
}

TEST_F(AbsoluteEncoderTest, CorrectLowerSoftLimits)
{
    ASSERT_EQ(this->encoder.getLowerSoftLimitIU(), this->lower_soft_limit);
}

TEST_F(AbsoluteEncoderTest, CorrectUpperSoftLimits)
{
    ASSERT_EQ(this->encoder.getUpperSoftLimitIU(), this->upper_soft_limit);
}

TEST_F(AbsoluteEncoderTest, ZeroPositionNotWithingLiits)
{
    ASSERT_THROW(
        march::AbsoluteEncoder(this->counts_per_rotation, motor_controller_type, this->upper_limit_iu,
            this->lower_limit_iu, 531695,this->lower_soft_limit_rad_diff, this->upper_soft_limit_rad_diff),
        march::error::HardwareException);
}

TEST_F(AbsoluteEncoderTest, LowerSoftLimitAboveUpperSoftLimit)
{
    ASSERT_THROW(
        march::AbsoluteEncoder(this->counts_per_rotation, motor_controller_type, this->upper_limit_iu,
            this->lower_limit_iu, this->zero_position_iu, this->lower_soft_limit_rad_diff, this->upper_soft_limit_rad_diff),
        march::error::HardwareException);
}

TEST_F(AbsoluteEncoderTest, LowerSoftLimitLowerThanLowerHardLimit)
{
    ASSERT_THROW(march::AbsoluteEncoder(this->counts_per_rotation, motor_controller_type, this->lower_limit_iu,
                     this->upper_limit_iu, this->zero_position_iu, -1.0, this->upper_soft_limit_rad_diff),
        march::error::HardwareException);
}

TEST_F(AbsoluteEncoderTest, UpperSoftLimitHigherThanUpperHardLimit)
{
    ASSERT_THROW(march::AbsoluteEncoder(this->counts_per_rotation, motor_controller_type, this->lower_limit_iu,
                     this->upper_limit_iu, this->zero_position_iu, this->lower_soft_limit_rad_diff, 5.0),
        march::error::HardwareException);
}

TEST_F(AbsoluteEncoderTest, IsWIthingHardlimitsTestTrue)
{
    ASSERT_TRUE(this->encoder.isWithinHardLimitsIU(540000));
}

TEST_F(AbsoluteEncoderTest, IsWIthingHardlimitsTestFalse)
{
    ASSERT_FALSE(this->encoder.isWithinHardLimitsIU(531695));
}

TEST_F(AbsoluteEncoderTest, IsWitinHardLimitsRadTrue)
{
    ASSERT_TRUE(this->encoder.isWithinHardLimitsRadians(1.0));
}

TEST_F(AbsoluteEncoderTest, IsWitinHardLimitsRadFalse)
{
    ASSERT_FALSE(this->encoder.isWithinHardLimitsRadians(-2.0));
}

TEST_F(AbsoluteEncoderTest, IsWithinSoftLimitsTestTrue)
{
    ASSERT_TRUE(this->encoder.isWithinSoftLimitsIU(540000));
}

TEST_F(AbsoluteEncoderTest, IsWithinSoftLimitsTestFalse)
{
    ASSERT_FALSE(this->encoder.isWithinSoftLimitsIU(531695));
}

TEST_F(AbsoluteEncoderTest, IsWitinSoftLimitsRadTrue)
{
    ASSERT_TRUE(this->encoder.isWithinSoftLimitsRadians(1.0));
}

TEST_F(AbsoluteEncoderTest, IsWitinSoftLimitsRadFalse)
{
    ASSERT_FALSE(this->encoder.isWithinSoftLimitsRadians(-2.0));
}

TEST_F(AbsoluteEncoderTest, IsValidTargetIUCurrentLowerThenSoftLimitTrue)
{
    ASSERT_TRUE(this->encoder.isValidTargetIU(this->lower_soft_limit - 1000, this->lower_soft_limit - 500));
}

TEST_F(AbsoluteEncoderTest, IsValidTargetIUCurrentLowerThenSoftLimitFalse)
{
    ASSERT_FALSE(this->encoder.isValidTargetIU(this->lower_soft_limit - 1000, this->lower_soft_limit - 1500));
}

TEST_F(AbsoluteEncoderTest, IsValidTargetIUCurrentHigherThenSoftLimitTrue)
{
    ASSERT_TRUE(this->encoder.isValidTargetIU(this->upper_soft_limit + 1000, this->upper_soft_limit + 500));
}

TEST_F(AbsoluteEncoderTest, IsValidTargetIUCurrentHigherThenSoftLimitFalse)
{
    ASSERT_FALSE(this->encoder.isValidTargetIU(this->upper_soft_limit + 1000, this->upper_soft_limit + 1500));
}

TEST_F(AbsoluteEncoderTest, IsValidTargetIUTrue)
{
    ASSERT_TRUE(this->encoder.isValidTargetIU(this->zero_position_iu, this->zero_position_iu + 500));
}

TEST_F(AbsoluteEncoderTest, ZeroPositionRadToZeroPosition)
{
    ASSERT_EQ(this->encoder.positionRadiansToIU(/*position=*/0.0), this->zero_position_iu);
}

TEST_F(AbsoluteEncoderTest, CorrectFromRad)
{
    const double radians = 1.0;
    const int32_t expected = (radians * this->counts_per_rotation / (2 * M_PI)) + this->zero_position_iu;
    ASSERT_EQ((int32_t)this->encoder.positionRadiansToIU(radians), expected);
}

TEST_F(AbsoluteEncoderTest, ZeroPositionToZeroRadians)
{
    ASSERT_DOUBLE_EQ(this->encoder.positionIUToRadians(this->zero_position_iu), 0.0);
}

TEST_F(AbsoluteEncoderTest, CorrectToRad)
{
    const int32_t iu = 1.0;
    const double expected = (iu - this->zero_position_iu) * 2 * M_PI / this->counts_per_rotation;
    ASSERT_EQ(this->encoder.positionIUToRadians(iu), expected);
}

#endif
