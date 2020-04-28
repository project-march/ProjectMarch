// Copyright 2018 Project March.
#include "mocks/MockPdoInterface.h"
#include "march_hardware/LowVoltage.h"

#include <sstream>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

class TestLowVoltage : public ::testing::Test
{
protected:
  MockPdoInterface mock_pdo;
};

TEST_F(TestLowVoltage, Equals)
{
  int slaveIndex = 1;
  NetMonitorOffsets netMonitoringOffsets;
  NetDriverOffsets netDriverOffsets;
  march::LowVoltage lowVoltage1(this->mock_pdo, slaveIndex, netMonitoringOffsets, netDriverOffsets);
  march::LowVoltage lowVoltage2(this->mock_pdo, slaveIndex, netMonitoringOffsets, netDriverOffsets);
  EXPECT_TRUE(lowVoltage1 == lowVoltage2);
}

TEST_F(TestLowVoltage, NotEquals)
{
  int slaveIndex = 1;
  NetMonitorOffsets netMonitoringOffsets;
  NetDriverOffsets netDriverOffsets;
  NetDriverOffsets netDriverOffsets2(1, 2, 3);
  march::LowVoltage lowVoltage1(this->mock_pdo, slaveIndex, netMonitoringOffsets, netDriverOffsets);
  march::LowVoltage lowVoltage2(this->mock_pdo, 2, netMonitoringOffsets, netDriverOffsets);
  march::LowVoltage lowVoltage3(this->mock_pdo, 2, netMonitoringOffsets, netDriverOffsets2);

  EXPECT_FALSE(lowVoltage1 == lowVoltage2);
  EXPECT_FALSE(lowVoltage1 == lowVoltage3);
  EXPECT_FALSE(lowVoltage2 == lowVoltage3);
}

TEST_F(TestLowVoltage, Stream)
{
  int slaveIndex = 1;
  NetMonitorOffsets netMonitoringOffsets;
  NetDriverOffsets netDriverOffsets;
  march::LowVoltage lowVoltage1(this->mock_pdo, slaveIndex, netMonitoringOffsets, netDriverOffsets);
  std::stringstream ss;
  ss << lowVoltage1;
  EXPECT_EQ("LowVoltage(slaveIndex: 1, netMonitoringOffsets: NetMonitorOffsets(powerDistributionBoardCurrent: -1, "
            "lowVoltageNet1Current: -1, lowVoltageNet2Current: -1, highVoltageNetCurrent: -1, lowVoltageState: -1, "
            "highVoltageOvercurrentTrigger: -1, highVoltageEnabled: -1, highVoltageState: -1), netDriverOffsets: "
            "NetDriverOffsets(lowVoltageNetOnOff: -1, highVoltageNetOnOff: -1, highVoltageNetEnableDisable: -1))",
            ss.str());
}
