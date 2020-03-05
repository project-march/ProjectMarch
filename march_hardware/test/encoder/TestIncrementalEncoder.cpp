// Copyright 2020 Project March.
#include "march_hardware/encoder/IncrementalEncoder.h"

#include <cmath>

#include <gtest/gtest.h>

class TestIncrementalEncoder : public testing::Test
{
protected:
  const size_t resolution = 12;
  const double transmission = 100;
  march::IncrementalEncoder encoder = march::IncrementalEncoder(this->resolution, this->transmission);
};

TEST_F(TestIncrementalEncoder, ZeroIUToRad)
{
  ASSERT_EQ(0.0, this->encoder.toRad(0));
}

TEST_F(TestIncrementalEncoder, CorrectToRad)
{
  const int32_t iu = 1000;
  const double expected = iu * this->transmission * 2.0 * M_PI / std::pow(2, this->resolution);
  ASSERT_EQ(expected, this->encoder.toRad(iu));
}
