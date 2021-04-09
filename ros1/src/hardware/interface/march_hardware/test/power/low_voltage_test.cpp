// Copyright 2018 Project March.
#include "../mocks/mock_pdo_interface.h"
#include "march_hardware/power/low_voltage.h"

#include <sstream>

#include <gtest/gtest.h>

class LowVoltageTest : public ::testing::Test {
protected:
    MockPdoInterfacePtr mock_pdo;
    march::PdoSlaveInterface pdo = march::PdoSlaveInterface(1, this->mock_pdo);
};

TEST_F(LowVoltageTest, Equals)
{
    NetMonitorOffsets netMonitoringOffsets;
    NetDriverOffsets netDriverOffsets;
    march::LowVoltage lowVoltage1(
        this->pdo, netMonitoringOffsets, netDriverOffsets);
    march::LowVoltage lowVoltage2(
        this->pdo, netMonitoringOffsets, netDriverOffsets);
    EXPECT_TRUE(lowVoltage1 == lowVoltage2);
}

TEST_F(LowVoltageTest, NotEquals)
{
    NetMonitorOffsets netMonitoringOffsets;
    NetDriverOffsets netDriverOffsets;
    NetDriverOffsets netDriverOffsets2(1, 2, 3);
    march::LowVoltage lowVoltage1(
        this->pdo, netMonitoringOffsets, netDriverOffsets);
    march::LowVoltage lowVoltage2(
        this->pdo, netMonitoringOffsets, netDriverOffsets2);

    EXPECT_FALSE(lowVoltage1 == lowVoltage2);
}

TEST_F(LowVoltageTest, Stream)
{
    NetMonitorOffsets netMonitoringOffsets;
    NetDriverOffsets netDriverOffsets;
    march::LowVoltage lowVoltage1(
        this->pdo, netMonitoringOffsets, netDriverOffsets);
    std::stringstream ss;
    ss << lowVoltage1;
    EXPECT_EQ("LowVoltage(netMonitoringOffsets: "
              "NetMonitorOffsets(powerDistributionBoardCurrent: -1, "
              "lowVoltageNet1Current: -1, lowVoltageNet2Current: -1, "
              "highVoltageNetCurrent: -1, lowVoltageState: -1, "
              "highVoltageOvercurrentTrigger: -1, highVoltageEnabled: -1, "
              "highVoltageState: -1), netDriverOffsets: "
              "NetDriverOffsets(lowVoltageNetOnOff: -1, highVoltageNetOnOff: "
              "-1, highVoltageNetEnableDisable: -1))",
        ss.str());
}
