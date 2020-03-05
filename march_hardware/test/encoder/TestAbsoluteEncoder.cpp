// Copyright 2020 Project March.
#include "march_hardware/encoder/AbsoluteEncoder.h"
#include "march_hardware/error/hardware_exception.h"

#include <cmath>
#include <tuple>

#include <gtest/gtest.h>

class TestAbsoluteEncoder : public testing::Test
{
protected:
  const size_t resolution = 17;
  const size_t total_positions = std::pow(2, resolution);
  const int32_t lower_limit = 2053;
  const int32_t upper_limit = 45617;
  const double lower_limit_rad = -0.34906585;
  const double upper_limit_rad = 1.745329252;
  const int32_t zero_position = lower_limit - lower_limit_rad * total_positions / (2 * M_PI);
  const double soft_buffer = 0.05;
  const double lower_soft_limit_rad = lower_limit_rad + soft_buffer;
  const double upper_soft_limit_rad = upper_limit_rad - soft_buffer;
  const int32_t lower_soft_limit = lower_soft_limit_rad * total_positions / (2 * M_PI) + zero_position;
  const int32_t upper_soft_limit = upper_soft_limit_rad * total_positions / (2 * M_PI) + zero_position;

  march::AbsoluteEncoder encoder = march::AbsoluteEncoder(resolution, lower_limit, upper_limit, lower_limit_rad,
                                                          upper_limit_rad, lower_soft_limit_rad, upper_soft_limit_rad);
};

class TestEncoderParameterizedLimits : public TestAbsoluteEncoder,
                                       public testing::WithParamInterface<std::tuple<int32_t, bool>>
{
};

class TestEncoderParameterizedSoftLimits : public TestAbsoluteEncoder,
                                           public testing::WithParamInterface<std::tuple<int32_t, bool>>
{
};

class TestEncoderParameterizedValidTarget : public TestAbsoluteEncoder,
                                            public testing::WithParamInterface<std::tuple<int32_t, int32_t, bool>>
{
};

TEST_F(TestAbsoluteEncoder, CorrectLowerHardLimits)
{
  ASSERT_EQ(this->encoder.getLowerHardLimitIU(), this->lower_limit);
}

TEST_F(TestAbsoluteEncoder, CorrectUpperHardLimit)
{
  ASSERT_EQ(this->encoder.getUpperHardLimitIU(), this->upper_limit);
}

TEST_F(TestAbsoluteEncoder, CorrectLowerSoftLimits)
{
  ASSERT_EQ(this->encoder.getLowerSoftLimitIU(), this->lower_soft_limit);
}

TEST_F(TestAbsoluteEncoder, CorrectUpperSoftLimits)
{
  ASSERT_EQ(this->encoder.getUpperSoftLimitIU(), this->upper_soft_limit);
}

TEST_F(TestAbsoluteEncoder, LowerSoftLimitAboveUpperSoftLimit)
{
  ASSERT_THROW(march::AbsoluteEncoder(this->resolution, this->lower_limit, this->upper_limit, this->lower_limit_rad,
                                      this->upper_limit_rad, this->upper_soft_limit_rad, this->lower_soft_limit_rad),
               march::error::HardwareException);
}

TEST_F(TestAbsoluteEncoder, LowerSoftLimitLowerThanLowerHardLimit)
{
  ASSERT_THROW(march::AbsoluteEncoder(this->resolution, this->lower_limit, this->upper_limit, this->lower_limit_rad,
                                      this->upper_limit_rad, -0.4, this->upper_soft_limit_rad),
               march::error::HardwareException);
}

TEST_F(TestAbsoluteEncoder, UpperSoftLimitHigherThanUpperHardLimit)
{
  ASSERT_THROW(march::AbsoluteEncoder(this->resolution, this->lower_limit, this->upper_limit, this->lower_limit_rad,
                                      this->upper_limit_rad, this->lower_soft_limit_rad, 2.0),
               march::error::HardwareException);
}

TEST_F(TestAbsoluteEncoder, ZeroPositionRadToZeroPosition)
{
  ASSERT_EQ(this->encoder.fromRad(0.0), this->zero_position);
}

TEST_F(TestAbsoluteEncoder, CorrectFromRad)
{
  const double radians = 1.0;
  const int32_t expected = (radians * this->total_positions / (2 * M_PI)) + this->zero_position;
  ASSERT_EQ(this->encoder.fromRad(radians), expected);
}

TEST_F(TestAbsoluteEncoder, ZeroPositionToZeroRadians)
{
  ASSERT_DOUBLE_EQ(this->encoder.toRad(this->zero_position), 0.0);
}

TEST_F(TestAbsoluteEncoder, CorrectToRad)
{
  const int32_t iu = 1.0;
  const double expected = (iu - this->zero_position) * 2 * M_PI / this->total_positions;
  ASSERT_EQ(this->encoder.toRad(iu), expected);
}

INSTANTIATE_TEST_CASE_P(ParameterizedLimits, TestEncoderParameterizedLimits,
                        testing::Values(std::make_tuple(2053 - 1, false), std::make_tuple(2053, false),
                                        std::make_tuple(2053 + 1, true), std::make_tuple(45617 - 1, true),
                                        std::make_tuple(45617, false), std::make_tuple(45617 + 1, false),
                                        std::make_tuple(3000, true), std::make_tuple(0, false),
                                        std::make_tuple(60001, false), std::make_tuple(-2020, false)));

TEST_P(TestEncoderParameterizedLimits, IsWithinHardLimits)
{
  const int32_t iu = std::get<0>(this->GetParam());
  const bool expected = std::get<1>(this->GetParam());
  ASSERT_EQ(this->encoder.isWithinHardLimitsIU(iu), expected);
}

INSTANTIATE_TEST_CASE_P(ParameterizedSoftLimits, TestEncoderParameterizedSoftLimits,
                        testing::Values(std::make_tuple(3095 - 1, false), std::make_tuple(3095, false),
                                        std::make_tuple(3095 + 1, true), std::make_tuple(44699 - 1, true),
                                        std::make_tuple(44699, false), std::make_tuple(44699 + 1, false),
                                        std::make_tuple(4500, true), std::make_tuple(0, false),
                                        std::make_tuple(60001, false), std::make_tuple(-101, false)));

TEST_P(TestEncoderParameterizedSoftLimits, IsWithinSoftLimits)
{
  const int32_t iu = std::get<0>(this->GetParam());
  const bool expected = std::get<1>(this->GetParam());
  ASSERT_EQ(this->encoder.isWithinSoftLimitsIU(iu), expected);
}

INSTANTIATE_TEST_CASE_P(ParameterizedValidTarget, TestEncoderParameterizedValidTarget,
                        testing::Values(std::make_tuple(4000, 4500, true), std::make_tuple(4000, 2000, false),
                                        std::make_tuple(46000, 40000, true), std::make_tuple(46000, 45900, true),
                                        std::make_tuple(44560, 44560, true), std::make_tuple(46000, 46001, false),
                                        std::make_tuple(46000, 2000, false), std::make_tuple(3000, 5000, true),
                                        std::make_tuple(3000, 3001, true), std::make_tuple(3144, 3144, true),
                                        std::make_tuple(0, -1, false), std::make_tuple(0, 60000, false)));

TEST_P(TestEncoderParameterizedValidTarget, IsValidTargetIU)
{
  const int32_t current_iu = std::get<0>(this->GetParam());
  const int32_t target_iu = std::get<1>(this->GetParam());
  const bool expected = std::get<2>(this->GetParam());
  ASSERT_EQ(this->encoder.isValidTargetIU(current_iu, target_iu), expected);
}
