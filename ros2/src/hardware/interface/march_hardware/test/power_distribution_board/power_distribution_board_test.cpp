#ifndef __clang_analyzer__
// NOLINTBEGIN
//
// Created by march on 19-1-23, Created by Marco Bak.
//
#include "../mocks/mock_slave.h"
#include "march_hardware/power_distribution_board/power_distribution_board.h"

#include <gmock/gmock.h>
#include <gtest/gtest.h>

using testing::Eq;
using testing::Return;

class PowerDistributionBoardTest : public ::testing::Test {
protected:
    MockPdoInterfacePtr mock_pdo = std::make_shared<MockPdoInterface>();
    MockSdoInterfacePtr mock_sdo = std::make_shared<MockSdoInterface>();
    MockSlave mock_slave = MockSlave(this->mock_pdo, this->mock_sdo);
};

// TEST_F(PowerDistributionBoardTest, Read)
//{
//    uint8_t expected_offset = 4;
//    march::PowerDistributionBoard distribution_board(mock_slave, /*byte_offset*/0);
//    march::PowerDistributionBoardData data = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
//    march::PowerDistributionBoardData expected_data = { 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14 };
//
//    for (int i = 0; i < 15; i++) {
//        march::bit32 bit;
//        bit.f = (float)i;
//        EXPECT_CALL(*this->mock_pdo, read32(Eq(this->mock_slave.getSlaveIndex()), expected_offset + i * 4))
//                .WillOnce(Return(bit));
//    }
////    ASSERT_NE(expected_data, data);
//    distribution_board.read32(data);
//    ASSERT_EQ(expected_data, data);
//}
// NOLINTEND
#endif
