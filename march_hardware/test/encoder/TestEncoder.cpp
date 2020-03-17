// Copyright 2020 Project March.
#include "../mocks/MockEncoder.cpp"
#include "march_hardware/error/hardware_exception.h"

#include <cmath>

#include <gtest/gtest.h>

/**
 * This test fixture uses the MockEncoder to test non virtual methods
 * of the Encoder abstract class. Otherwise it is not possible to
 * instantiate a pure abstract class. So as long as the non virtual
 * methods are tested all is ok.
 */
class TestEncoder : public testing::Test
{
protected:
  const size_t resolution = 12;
};

TEST_F(TestEncoder, ResolutionBelowRange)
{
  ASSERT_THROW(MockEncoder(0), march::error::HardwareException);
}

TEST_F(TestEncoder, ResolutionAboveRange)
{
  ASSERT_THROW(MockEncoder(50), march::error::HardwareException);
}

TEST_F(TestEncoder, SetSlaveIndex)
{
  MockEncoder encoder(this->resolution);
  const int expected = 10;
  encoder.setSlaveIndex(expected);
  ASSERT_EQ(expected, encoder.getSlaveIndex());
}

TEST_F(TestEncoder, CorrectTotalPositions)
{
  MockEncoder encoder(this->resolution);
  ASSERT_EQ(std::pow(2, this->resolution), encoder.getTotalPositions());
}
