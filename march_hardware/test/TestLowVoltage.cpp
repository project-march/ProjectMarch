// Copyright 2018 Project March.

#include "march_hardware/LowVoltage.h"
#include "gtest/gtest.h"
#include <gmock/gmock.h>
#include <sstream>

using ::testing::Return;
using ::testing::AtLeast;
using ::testing::AtMost;

class TestLowVoltage : public ::testing::Test
{
protected:
};

TEST_F(TestLowVoltage, Equals)
{
  int slaveIndex = 1;
  NetMonitorOffsets netMonitoringOffsets;
  NetDriverOffsets netDriverOffsets;
  march4cpp::LowVoltage lowVoltage1(slaveIndex, netMonitoringOffsets, netDriverOffsets);
  march4cpp::LowVoltage lowVoltage2(slaveIndex, netMonitoringOffsets, netDriverOffsets);
  EXPECT_TRUE(lowVoltage1 == lowVoltage2);
}

TEST_F(TestLowVoltage, NotEquals)
{
  int slaveIndex = 1;
  NetMonitorOffsets netMonitoringOffsets;
  NetDriverOffsets netDriverOffsets;
  NetDriverOffsets netDriverOffsets2(1, 2, 3);
  march4cpp::LowVoltage lowVoltage1(slaveIndex, netMonitoringOffsets, netDriverOffsets);
  march4cpp::LowVoltage lowVoltage2(2, netMonitoringOffsets, netDriverOffsets);
  march4cpp::LowVoltage lowVoltage3(2, netMonitoringOffsets, netDriverOffsets2);

  EXPECT_FALSE(lowVoltage1 == lowVoltage2);
  EXPECT_FALSE(lowVoltage1 == lowVoltage3);
  EXPECT_FALSE(lowVoltage2 == lowVoltage3);
}

TEST_F(TestLowVoltage, Stream)
{
  int slaveIndex = 1;
  NetMonitorOffsets netMonitoringOffsets;
  NetDriverOffsets netDriverOffsets;
  march4cpp::LowVoltage lowVoltage1(slaveIndex, netMonitoringOffsets, netDriverOffsets);
  std::stringstream ss;
  ss << lowVoltage1;
  EXPECT_EQ("LowVoltage(slaveIndex: 1, netMonitoringOffsets: NetMonitorOffsets(powerDistributionBoardCurrent: -1, "
            "lowVoltageNet1Current: -1, lowVoltageNet2Current: -1, highVoltageNetCurrent: -1, lowVoltageState: -1, "
            "highVoltageOvercurrentTrigger: -1, highVoltageEnabled: -1, highVoltageState: -1), netDriverOffsets: "
            "NetDriverOffsets(lowVoltageNetOnOff: -1, highVoltageNetOnOff: -1, allHighVoltageOnOff: -1))",
            ss.str());
}