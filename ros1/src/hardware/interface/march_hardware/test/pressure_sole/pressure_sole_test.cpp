// Copyright 2018 Project March.
#include "../mocks/mock_slave.h"
#include "march_hardware/pressure_sole/pressure_sole.h"

#include <gmock/gmock.h>
#include <gtest/gtest.h>

using testing::Eq;
using testing::Return;

class PressureSoleTest : public ::testing::Test {
protected:
    MockPdoInterfacePtr mock_pdo = std::make_shared<MockPdoInterface>();
    MockSdoInterfacePtr mock_sdo = std::make_shared<MockSdoInterface>();
    MockSlave mock_slave = MockSlave(this->mock_pdo, this->mock_sdo);
};

TEST_F(PressureSoleTest, GetSide)
{
    march::PressureSole pressure_sole(mock_slave, 0, "left");

    ASSERT_EQ("left", pressure_sole.getSide());
}

TEST_F(PressureSoleTest, PressureSoleDataEqTrue)
{
    march::PressureSoleData data_1 = { 0, 1, 2, 3, 4, 5, 6, 7 };
    march::PressureSoleData data_2 = { 0, 1, 2, 3, 4, 5, 6, 7 };

    ASSERT_EQ(data_1, data_2);
}

TEST_F(PressureSoleTest, PressureSoleDataEqFalse)
{
    march::PressureSoleData data_1 = { 0, 1, 2, 3, 5, 6, 7, 8 };
    march::PressureSoleData data_2 = { 0, 1, 2, 3, 4, 5, 6, 7 };

    ASSERT_FALSE(data_1 == data_2);
}

TEST_F(PressureSoleTest, Read)
{
    uint8_t expected_offset = 4;
    march::PressureSole pressure_sole(mock_slave, expected_offset, "left");
    march::PressureSoleData expected_data = { 0, 1, 2, 3, 4, 5, 6, 7 };

    for (int i = 0; i < 8; i++) {
        EXPECT_CALL(*this->mock_pdo,
            read32(
                Eq(this->mock_slave.getSlaveIndex()), expected_offset + i * 4))
            .WillOnce(Return(march::bit32 { .f = (float)i }));
    }

    ASSERT_EQ(expected_data, pressure_sole.read());
}
