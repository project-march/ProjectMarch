#ifndef __clang_analyzer__
// NOLINTBEGIN
// Copyright 2020 Project March.
#include "../mocks/mock_encoder.h"
#include "../mocks/mock_pdo_interface.h"
#include "march_hardware/error/hardware_exception.h"

#include <cmath>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

using testing::Eq;
using testing::Return;

/**
 * This test fixture uses the MockEncoder to test non virtual methods
 * of the Encoder abstract class. Otherwise it is not possible to
 * instantiate a pure abstract class. So as long as the non virtual
 * methods are tested all is ok.
 */
class EncoderTest : public testing::Test {
protected:
    const uint16_t slave_index = 1;
    const size_t counts_per_rotation = 1 << 12;
};

TEST_F(EncoderTest, CountsPerRotationBelowRange)
{
    ASSERT_THROW(MockEncoder((size_t)1 << 0), march::error::HardwareException);
}

TEST_F(EncoderTest, CountsPerRotationAboveRange)
{
    ASSERT_THROW(MockEncoder((size_t)1 << 50), march::error::HardwareException);
}

TEST_F(EncoderTest, CorrectTotalPositions)
{
    MockEncoder encoder(this->counts_per_rotation);
    ASSERT_EQ(counts_per_rotation, encoder.getTotalPositions());
}

TEST_F(EncoderTest, CorrectGetDirectionTest)
{
    MockEncoder encoder(this->counts_per_rotation);
    ASSERT_EQ(march::Encoder::Direction::Positive, encoder.getDirection());
}
// NOLINTEND
#endif
