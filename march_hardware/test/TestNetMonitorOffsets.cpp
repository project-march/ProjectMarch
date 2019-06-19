// Copyright 2018 Project March.

#include "march_hardware/NetMonitorOffsets.h"
#include "gtest/gtest.h"
#include <gmock/gmock.h>
#include <sstream>

using ::testing::Return;
using ::testing::AtLeast;
using ::testing::AtMost;

class TestNetMonitorOffsets : public ::testing::Test
{
protected:
  const int powerDistributionBoardCurrent = 1;
  const int lowVoltageNet1Current = 2;
  const int lowVoltageNet2Current = 3;
  const int highVoltageNetCurrent = 4;
  const int lowVoltageState = 5;
  const int highVoltageOvercurrentTrigger = 6;
  const int highVoltageEnabled = 7;
  const int highVoltageState = 8;
};
TEST_F(TestNetMonitorOffsets, ValidValues)
{
  NetMonitorOffsets netMonitorOffsets(powerDistributionBoardCurrent, lowVoltageNet1Current, lowVoltageNet2Current,
                                      highVoltageNetCurrent, lowVoltageState, highVoltageOvercurrentTrigger,
                                      highVoltageEnabled, highVoltageState);

  EXPECT_EQ(powerDistributionBoardCurrent, netMonitorOffsets.getPowerDistributionBoardCurrent());
  EXPECT_EQ(lowVoltageNet1Current, netMonitorOffsets.getLowVoltageNetCurrent(1));
  EXPECT_EQ(lowVoltageNet2Current, netMonitorOffsets.getLowVoltageNetCurrent(2));
  EXPECT_EQ(highVoltageNetCurrent, netMonitorOffsets.getHighVoltageNetCurrent());
  EXPECT_EQ(lowVoltageState, netMonitorOffsets.getLowVoltageState());
  EXPECT_EQ(highVoltageOvercurrentTrigger, netMonitorOffsets.getHighVoltageOvercurrentTrigger());
  EXPECT_EQ(highVoltageEnabled, netMonitorOffsets.getHighVoltageEnabled());
  EXPECT_EQ(highVoltageState, netMonitorOffsets.getHighVoltageState());
}
TEST_F(TestNetMonitorOffsets, EmptyConstructor)
{
  NetMonitorOffsets netMonitorOffsets;

  EXPECT_THROW(netMonitorOffsets.getPowerDistributionBoardCurrent(), std::runtime_error);
  EXPECT_THROW(netMonitorOffsets.getLowVoltageNetCurrent(1), std::runtime_error);
  EXPECT_THROW(netMonitorOffsets.getLowVoltageNetCurrent(2), std::runtime_error);
  EXPECT_THROW(netMonitorOffsets.getHighVoltageNetCurrent(), std::runtime_error);
  EXPECT_THROW(netMonitorOffsets.getLowVoltageState(), std::runtime_error);
  EXPECT_THROW(netMonitorOffsets.getHighVoltageOvercurrentTrigger(), std::runtime_error);
  EXPECT_THROW(netMonitorOffsets.getHighVoltageEnabled(), std::runtime_error);
  EXPECT_THROW(netMonitorOffsets.getHighVoltageState(), std::runtime_error);
}

TEST_F(TestNetMonitorOffsets, GetNotExistingNet)
{
  NetMonitorOffsets netMonitorOffsets(powerDistributionBoardCurrent, lowVoltageNet1Current, lowVoltageNet2Current,
                                      highVoltageNetCurrent, lowVoltageState, highVoltageOvercurrentTrigger,
                                      highVoltageEnabled, highVoltageState);
  EXPECT_THROW(netMonitorOffsets.getLowVoltageNetCurrent(42), std::runtime_error);
}

TEST_F(TestNetMonitorOffsets, InValidValues)
{
  EXPECT_THROW(NetMonitorOffsets netMonitorOffsets(powerDistributionBoardCurrent, lowVoltageNet1Current,
                                                   lowVoltageNet2Current, highVoltageNetCurrent, lowVoltageState,
                                                   highVoltageOvercurrentTrigger, highVoltageEnabled, -1),
               std::runtime_error);
}

TEST_F(TestNetMonitorOffsets, Equals)
{
  NetMonitorOffsets netMonitorOffsets1(powerDistributionBoardCurrent, lowVoltageNet1Current, lowVoltageNet2Current,
                                       highVoltageNetCurrent, lowVoltageState, highVoltageOvercurrentTrigger,
                                       highVoltageEnabled, highVoltageState);
  NetMonitorOffsets netMonitorOffsets2(powerDistributionBoardCurrent, lowVoltageNet1Current, lowVoltageNet2Current,
                                       highVoltageNetCurrent, lowVoltageState, highVoltageOvercurrentTrigger,
                                       highVoltageEnabled, highVoltageState);
  EXPECT_TRUE(netMonitorOffsets1 == netMonitorOffsets2);
}
TEST_F(TestNetMonitorOffsets, NotEquals)
{
  NetMonitorOffsets netMonitorOffsets1(24, lowVoltageNet1Current, lowVoltageNet2Current, highVoltageNetCurrent,
                                       lowVoltageState, highVoltageOvercurrentTrigger, highVoltageEnabled,
                                       highVoltageState);
  NetMonitorOffsets netMonitorOffsets2(powerDistributionBoardCurrent, lowVoltageNet1Current, lowVoltageNet2Current,
                                       highVoltageNetCurrent, 42, highVoltageOvercurrentTrigger, highVoltageEnabled,
                                       highVoltageState);
  NetMonitorOffsets netMonitorOffsets3(powerDistributionBoardCurrent, lowVoltageNet1Current, lowVoltageNet2Current,
                                       highVoltageNetCurrent, 42, highVoltageOvercurrentTrigger, highVoltageEnabled,
                                       11);

  EXPECT_FALSE(netMonitorOffsets1 == netMonitorOffsets2);
  EXPECT_FALSE(netMonitorOffsets1 == netMonitorOffsets3);
  EXPECT_FALSE(netMonitorOffsets2 == netMonitorOffsets3);
}

TEST_F(TestNetMonitorOffsets, TestStream)
{
  NetMonitorOffsets netMonitorOffsets(powerDistributionBoardCurrent, lowVoltageNet1Current, lowVoltageNet2Current,
                                      highVoltageNetCurrent, lowVoltageState, highVoltageOvercurrentTrigger,
                                      highVoltageEnabled, highVoltageState);
  std::stringstream ss;
  ss << "netMonitorOffsets: " << netMonitorOffsets;
  EXPECT_EQ("netMonitorOffsets: NetMonitorOffsets(powerDistributionBoardCurrent: 1, lowVoltageNet1Current: 2, "
            "lowVoltageNet2Current: 3, highVoltageNetCurrent: 4, lowVoltageState: 5, highVoltageOvercurrentTrigger: 6, "
            "highVoltageEnabled: 7, highVoltageState: 8)",
            ss.str());
}