// Copyright 2018 Project March.
#include "../mocks/mock_slave.h"
#include "march_hardware/temperature/temperature_ges.h"

#include <gmock/gmock.h>
#include <gtest/gtest.h>

using testing::Eq;
using testing::Return;

class TemperatureGESTest : public ::testing::Test {
protected:
    MockPdoInterfacePtr mock_pdo = std::make_shared<MockPdoInterface>();
    MockSdoInterfacePtr mock_sdo = std::make_shared<MockSdoInterface>();
    MockSlave mock_slave = MockSlave(this->mock_pdo, this->mock_sdo);
};

TEST_F(TemperatureGESTest, ByteOffsetOne)
{
    ASSERT_NO_THROW(march::TemperatureGES(this->mock_slave, 1));
}

TEST_F(TemperatureGESTest, ByteOffsetZero)
{
    ASSERT_NO_THROW(march::TemperatureGES(this->mock_slave, 0));
}

TEST_F(TemperatureGESTest, GetTemperature)
{
    const uint8_t expected_offset = 3;
    const float temperature = 1.0;
    EXPECT_CALL(*this->mock_pdo,
        read32(Eq(this->mock_slave.getSlaveIndex()), Eq(expected_offset)))
        .WillOnce(Return(march::bit32 { .f = temperature }));

    const march::TemperatureGES ges(this->mock_slave, expected_offset);

    ASSERT_FLOAT_EQ(ges.getTemperature(), temperature);
}
