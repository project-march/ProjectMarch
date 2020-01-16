// Copyright 2018 Project March.

#include "march_hardware/PowerDistributionBoard.h"
#include "gtest/gtest.h"
#include <gmock/gmock.h>
#include <sstream>

using ::testing::AtLeast;
using ::testing::AtMost;
using ::testing::Return;

class TestPowerDistributionBoard : public ::testing::Test
{
protected:
  NetMonitorOffsets netMonitoringOffsets;
  NetDriverOffsets netDriverOffsets;
  BootShutdownOffsets bootShutdownOffsets;
  int slaveIndex = 2;
};

TEST_F(TestPowerDistributionBoard, Unequals)
{
  NetDriverOffsets netDriverOffsets2(1, 2, 3);
  NetMonitorOffsets netMonitorOffsets2(1, 1, 1, 1, 1, 1, 1, 1);
  march::PowerDistributionBoard powerDistributionBoard1(slaveIndex, netMonitoringOffsets, netDriverOffsets,
                                                        bootShutdownOffsets);
  march::PowerDistributionBoard powerDistributionBoard2(3, netMonitoringOffsets, netDriverOffsets, bootShutdownOffsets);
  march::PowerDistributionBoard powerDistributionBoard3(slaveIndex, netMonitoringOffsets, netDriverOffsets2,
                                                        bootShutdownOffsets);
  march::PowerDistributionBoard powerDistributionBoard4(slaveIndex, netMonitorOffsets2, netDriverOffsets,
                                                        bootShutdownOffsets);

  EXPECT_FALSE(powerDistributionBoard1 == powerDistributionBoard2);
  EXPECT_FALSE(powerDistributionBoard1 == powerDistributionBoard3);
  EXPECT_FALSE(powerDistributionBoard1 == powerDistributionBoard4);
  EXPECT_FALSE(powerDistributionBoard1 == powerDistributionBoard3);
  EXPECT_FALSE(powerDistributionBoard4 == powerDistributionBoard2);
}

TEST_F(TestPowerDistributionBoard, Equals)
{
  march::PowerDistributionBoard powerDistributionBoard1(slaveIndex, netMonitoringOffsets, netDriverOffsets,
                                                        bootShutdownOffsets);
  march::PowerDistributionBoard powerDistributionBoard2(slaveIndex, netMonitoringOffsets, netDriverOffsets,
                                                        bootShutdownOffsets);
  EXPECT_TRUE(powerDistributionBoard1 == powerDistributionBoard2);
}

TEST_F(TestPowerDistributionBoard, Stream)
{
  march::PowerDistributionBoard powerDistributionBoard1(slaveIndex, netMonitoringOffsets, netDriverOffsets,
                                                        bootShutdownOffsets);
  std::stringstream ss;
  ss << powerDistributionBoard1;
  EXPECT_EQ("PowerDistributionBoard(slaveIndex: 2, masterOnlineToggle: 0, HighVoltage(slaveIndex: 2, "
            "netMonitoringOffsets: NetMonitorOffsets(powerDistributionBoardCurrent: -1, lowVoltageNet1Current: -1, "
            "lowVoltageNet2Current: -1, highVoltageNetCurrent: -1, lowVoltageState: -1, highVoltageOvercurrentTrigger: "
            "-1, highVoltageEnabled: -1, highVoltageState: -1), netDriverOffsets: NetDriverOffsets(lowVoltageNetOnOff: "
            "-1, highVoltageNetOnOff: -1, highVoltageNetEnableDisable: -1)), LowVoltage(slaveIndex: 2, "
            "netMonitoringOffsets: NetMonitorOffsets(powerDistributionBoardCurrent: -1, lowVoltageNet1Current: -1, "
            "lowVoltageNet2Current: -1, highVoltageNetCurrent: -1, lowVoltageState: -1, highVoltageOvercurrentTrigger: "
            "-1, highVoltageEnabled: -1, highVoltageState: -1), netDriverOffsets: NetDriverOffsets(lowVoltageNetOnOff: "
            "-1, highVoltageNetOnOff: -1, highVoltageNetEnableDisable: -1)))",
            ss.str());
}
