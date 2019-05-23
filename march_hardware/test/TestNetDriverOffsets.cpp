// Copyright 2018 Project March.

#include "march_hardware/NetDriverOffsets.h"
#include "gtest/gtest.h"
#include <gmock/gmock.h>
#include <sstream>

using ::testing::Return;
using ::testing::AtLeast;
using ::testing::AtMost;

class TestNetDriverOffsets : public ::testing::Test
{
protected:
  const int lowVoltageNetOnOff = 1;
  const int highVoltageNetOnOff = 1;
  const int highVoltageEmergencySwitchOnOff = 1;
};

TEST_F(TestNetDriverOffsets, ValidValues)
{
  NetDriverOffsets netDriverOffsets(lowVoltageNetOnOff, highVoltageNetOnOff, highVoltageEmergencySwitchOnOff);

  EXPECT_EQ(lowVoltageNetOnOff, netDriverOffsets.getLowVoltageNetOnOff());
  EXPECT_EQ(highVoltageNetOnOff, netDriverOffsets.getHighVoltageNetOnOff());
  EXPECT_EQ(highVoltageEmergencySwitchOnOff, netDriverOffsets.getHighVoltageEmergencySwitchOnOff());
}
TEST_F(TestNetDriverOffsets, EmptyConstructor)
{
  NetDriverOffsets netDriverOffsets;
  EXPECT_THROW(netDriverOffsets.getLowVoltageNetOnOff(), std::runtime_error);
  EXPECT_THROW(netDriverOffsets.getHighVoltageNetOnOff(), std::runtime_error);
  EXPECT_THROW(netDriverOffsets.getHighVoltageEmergencySwitchOnOff(), std::runtime_error);
}

TEST_F(TestNetDriverOffsets, InValidValues)
{
  EXPECT_THROW(NetDriverOffsets netDriverOffsets(lowVoltageNetOnOff, -7, highVoltageEmergencySwitchOnOff),
               std::runtime_error);
  EXPECT_THROW(NetDriverOffsets netDriverOffsets(-1, highVoltageNetOnOff, highVoltageEmergencySwitchOnOff),
               std::runtime_error);
  EXPECT_THROW(NetDriverOffsets netDriverOffsets(lowVoltageNetOnOff, highVoltageNetOnOff, -12), std::runtime_error);
}

TEST_F(TestNetDriverOffsets, Equals)
{
  NetDriverOffsets netDriverOffsets1(lowVoltageNetOnOff, highVoltageNetOnOff, highVoltageEmergencySwitchOnOff);
  NetDriverOffsets netDriverOffsets2(lowVoltageNetOnOff, highVoltageNetOnOff, highVoltageEmergencySwitchOnOff);

  EXPECT_TRUE(netDriverOffsets1 == netDriverOffsets2);
}
TEST_F(TestNetDriverOffsets, NotEquals)
{
  NetDriverOffsets netDriverOffsets1(11, highVoltageNetOnOff, highVoltageEmergencySwitchOnOff);
  NetDriverOffsets netDriverOffsets2(lowVoltageNetOnOff, 22, highVoltageEmergencySwitchOnOff);
  NetDriverOffsets netDriverOffsets3(lowVoltageNetOnOff, highVoltageNetOnOff, 33);

  EXPECT_FALSE(netDriverOffsets1 == netDriverOffsets2);
  EXPECT_FALSE(netDriverOffsets1 == netDriverOffsets3);
  EXPECT_FALSE(netDriverOffsets2 == netDriverOffsets3);
}

TEST_F(TestNetDriverOffsets, TestStream)
{
  NetDriverOffsets netDriverOffsets(lowVoltageNetOnOff, highVoltageNetOnOff, highVoltageEmergencySwitchOnOff);
  std::stringstream ss;
  ss << netDriverOffsets;
  EXPECT_EQ("NetDriverOffsets(lowVoltageNetOnOff: 1, highVoltageNetOnOff: 1, highVoltageEmergencySwitchOnOff: 1)", ss.str());
}