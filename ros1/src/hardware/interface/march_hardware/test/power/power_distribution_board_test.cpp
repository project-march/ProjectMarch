// Copyright 2018 Project March.
#include "../mocks/mock_slave.h"
#include "march_hardware/power/power_distribution_board.h"

#include <sstream>

#include <gtest/gtest.h>

class PowerDistributionBoardTest : public ::testing::Test {
protected:
    MockPdoInterfacePtr mock_pdo = std::make_shared<MockPdoInterface>();
    MockSdoInterfacePtr mock_sdo = std::make_shared<MockSdoInterface>();
    MockSlave mock_slave = MockSlave(this->mock_pdo, this->mock_sdo);
    NetMonitorOffsets netMonitoringOffsets;
    NetDriverOffsets netDriverOffsets;
    BootShutdownOffsets bootShutdownOffsets;
    int slaveIndex = 2;
};

TEST_F(PowerDistributionBoardTest, Unequals)
{
    NetDriverOffsets netDriverOffsets2(1, 2, 3);
    NetMonitorOffsets netMonitorOffsets2(1, 1, 1, 1, 1, 1, 1, 1);
    march::PowerDistributionBoard powerDistributionBoard1(this->mock_slave,
        netMonitoringOffsets, netDriverOffsets, bootShutdownOffsets);
    march::PowerDistributionBoard powerDistributionBoard2(this->mock_slave,
        netMonitoringOffsets, netDriverOffsets2, bootShutdownOffsets);
    march::PowerDistributionBoard powerDistributionBoard3(this->mock_slave,
        netMonitorOffsets2, netDriverOffsets, bootShutdownOffsets);

    EXPECT_FALSE(powerDistributionBoard1 == powerDistributionBoard2);
    EXPECT_FALSE(powerDistributionBoard1 == powerDistributionBoard3);
    EXPECT_FALSE(powerDistributionBoard2 == powerDistributionBoard3);
}

TEST_F(PowerDistributionBoardTest, Equals)
{
    march::PowerDistributionBoard powerDistributionBoard1(this->mock_slave,
        netMonitoringOffsets, netDriverOffsets, bootShutdownOffsets);
    march::PowerDistributionBoard powerDistributionBoard2(this->mock_slave,
        netMonitoringOffsets, netDriverOffsets, bootShutdownOffsets);
    EXPECT_TRUE(powerDistributionBoard1 == powerDistributionBoard2);
}
