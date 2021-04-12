// Copyright 2020 Project March.
#include "march_hardware/encoder/incremental_encoder.h"

#include <cmath>

#include <gtest/gtest.h>

class IncrementalEncoderTest : public testing::Test {
protected:
    const size_t resolution = 12;
    const double transmission = 100;
    march::IncrementalEncoder encoder
        = march::IncrementalEncoder(this->resolution, this->transmission);
};

TEST_F(IncrementalEncoderTest, ZeroIUToRad)
{
    ASSERT_EQ(0.0, this->encoder.toRad(0));
}

TEST_F(IncrementalEncoderTest, CorrectToRad)
{
    const int32_t iu = 1000;
    const double expected = iu * 2.0 * M_PI
        / (std::pow(2, this->resolution) * this->transmission);
    ASSERT_DOUBLE_EQ(expected, this->encoder.toRad(iu));
}
