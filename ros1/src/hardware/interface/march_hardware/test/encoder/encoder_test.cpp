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
    const size_t resolution = 12;
};

TEST_F(EncoderTest, ResolutionBelowRange)
{
    ASSERT_THROW(MockEncoder(0), march::error::HardwareException);
}

TEST_F(EncoderTest, ResolutionAboveRange)
{
    ASSERT_THROW(MockEncoder(50), march::error::HardwareException);
}

TEST_F(EncoderTest, GetAngleIU)
{
    MockEncoder encoder(this->resolution);
    const int32_t expected = 101;
    const uint8_t expected_offset = 4;
    march::bit32 result;
    result.i = expected;
    MockPdoInterfacePtr mock_pdo = std::make_shared<MockPdoInterface>();
    march::PdoSlaveInterface pdo(this->slave_index, mock_pdo);

    EXPECT_CALL(*mock_pdo, read32(this->slave_index, Eq(expected_offset)))
        .WillOnce(Return(result));

    ASSERT_EQ(expected, encoder.getAngleIU(pdo, expected_offset));
}

TEST_F(EncoderTest, CorrectTotalPositions)
{
    MockEncoder encoder(this->resolution);
    ASSERT_EQ(std::pow(2, this->resolution), encoder.getTotalPositions());
}
