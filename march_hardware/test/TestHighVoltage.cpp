// Copyright 2018 Project March.
#include "mocks/MockPdoInterface.h"
#include "march_hardware/HighVoltage.h"

#include <gtest/gtest.h>
#include <sstream>

class TestHighVoltage : public ::testing::Test
{
protected:
  MockPdoInterface mock_pdo;
};

TEST_F(TestHighVoltage, Equals)
{
  int slaveIndex = 2;
  NetMonitorOffsets netMonitoringOffsets;
  NetDriverOffsets netDriverOffsets;
  march::HighVoltage highVoltage1(this->mock_pdo, slaveIndex, netMonitoringOffsets, netDriverOffsets);
  march::HighVoltage highVoltage2(this->mock_pdo, slaveIndex, netMonitoringOffsets, netDriverOffsets);
  EXPECT_TRUE(highVoltage1 == highVoltage2);
}

TEST_F(TestHighVoltage, UnEqual)
{
  int slaveIndex = 2;
  NetMonitorOffsets netMonitoringOffsets;
  NetDriverOffsets netDriverOffsets1;
  NetDriverOffsets netDriverOffsets2(1, 2, 3);
  march::HighVoltage highVoltage1(this->mock_pdo, 4, netMonitoringOffsets, netDriverOffsets1);
  march::HighVoltage highVoltage2(this->mock_pdo, slaveIndex, netMonitoringOffsets, netDriverOffsets1);
  march::HighVoltage highVoltage3(this->mock_pdo, slaveIndex, netMonitoringOffsets, netDriverOffsets2);
  EXPECT_FALSE(highVoltage1 == highVoltage2);
  EXPECT_FALSE(highVoltage1 == highVoltage3);
  EXPECT_FALSE(highVoltage2 == highVoltage3);
}

TEST_F(TestHighVoltage, Stream)
{
  int slaveIndex = 1;
  NetMonitorOffsets netMonitoringOffsets;
  NetDriverOffsets netDriverOffsets;
  march::HighVoltage highVoltage(this->mock_pdo, slaveIndex, netMonitoringOffsets, netDriverOffsets);
  std::stringstream ss;
  ss << highVoltage;
  EXPECT_EQ("HighVoltage(slaveIndex: 1, netMonitoringOffsets: NetMonitorOffsets(powerDistributionBoardCurrent: -1, "
            "lowVoltageNet1Current: -1, lowVoltageNet2Current: -1, highVoltageNetCurrent: -1, lowVoltageState: -1, "
            "highVoltageOvercurrentTrigger: -1, highVoltageEnabled: -1, highVoltageState: -1), netDriverOffsets: "
            "NetDriverOffsets(lowVoltageNetOnOff: -1, highVoltageNetOnOff: -1, highVoltageNetEnableDisable: -1))",
            ss.str());
}
