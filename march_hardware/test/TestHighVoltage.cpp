// Copyright 2018 Project March.

#include "march_hardware/HighVoltage.h"
#include "gtest/gtest.h"
#include <gmock/gmock.h>
#include <sstream>

using ::testing::Return;
using ::testing::AtLeast;
using ::testing::AtMost;

class TestHighVoltage : public ::testing::Test
{
protected:
};

TEST_F(TestHighVoltage, Equals)
{
  int slaveIndex = 2;
  NetMonitorOffsets netMonitoringOffsets;
  NetDriverOffsets netDriverOffsets;
  march4cpp::HighVoltage highVoltage1(slaveIndex, netMonitoringOffsets, netDriverOffsets);
  march4cpp::HighVoltage highVoltage2(slaveIndex, netMonitoringOffsets, netDriverOffsets);
  EXPECT_TRUE(highVoltage1 == highVoltage2);
}

TEST_F(TestHighVoltage, UnEqual)
{
  int slaveIndex = 2;
  NetMonitorOffsets netMonitoringOffsets;
  NetDriverOffsets netDriverOffsets1;
  NetDriverOffsets netDriverOffsets2(1, 2, 3);
  march4cpp::HighVoltage highVoltage1(4, netMonitoringOffsets, netDriverOffsets1);
  march4cpp::HighVoltage highVoltage2(slaveIndex, netMonitoringOffsets, netDriverOffsets1);
  march4cpp::HighVoltage highVoltage3(slaveIndex, netMonitoringOffsets, netDriverOffsets2);
  EXPECT_FALSE(highVoltage1 == highVoltage2);
  EXPECT_FALSE(highVoltage1 == highVoltage3);
  EXPECT_FALSE(highVoltage2 == highVoltage3);
}

TEST_F(TestHighVoltage, Stream)
{
  int slaveIndex = 1;
  NetMonitorOffsets netMonitoringOffsets;
  NetDriverOffsets netDriverOffsets;
  march4cpp::HighVoltage highVoltage(slaveIndex, netMonitoringOffsets, netDriverOffsets);
  std::stringstream ss;
  ss << highVoltage;
  EXPECT_EQ("HighVoltage(slaveIndex: 1, netMonitoringOffsets: NetMonitorOffsets(powerDistributionBoardCurrent: -1, "
            "lowVoltageNet1Current: -1, lowVoltageNet2Current: -1, highVoltageNetCurrent: -1, lowVoltageState: -1, "
            "highVoltageOvercurrentTrigger: -1, highVoltageEnabled: -1, highVoltageState: -1), netDriverOffsets: "
            "NetDriverOffsets(lowVoltageNetOnOff: -1, highVoltageNetOnOff: -1, highVoltageEmergencySwitchOnOff: -1))",
            ss.str());
}