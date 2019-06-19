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
  const int allHighVoltageOnOff = 1;
};

TEST_F(TestNetDriverOffsets, ValidValues)
{
  NetDriverOffsets netDriverOffsets(lowVoltageNetOnOff, highVoltageNetOnOff, allHighVoltageOnOff);

  EXPECT_EQ(lowVoltageNetOnOff, netDriverOffsets.getLowVoltageNetOnOff());
  EXPECT_EQ(highVoltageNetOnOff, netDriverOffsets.getHighVoltageNetOnOff());
  EXPECT_EQ(allHighVoltageOnOff, netDriverOffsets.getAllHighVoltageOnOff());
}
TEST_F(TestNetDriverOffsets, EmptyConstructor)
{
  NetDriverOffsets netDriverOffsets;
  EXPECT_THROW(netDriverOffsets.getLowVoltageNetOnOff(), std::runtime_error);
  EXPECT_THROW(netDriverOffsets.getHighVoltageNetOnOff(), std::runtime_error);
  EXPECT_THROW(netDriverOffsets.getAllHighVoltageOnOff(), std::runtime_error);
}

TEST_F(TestNetDriverOffsets, InValidValues)
{
  EXPECT_THROW(NetDriverOffsets netDriverOffsets(lowVoltageNetOnOff, -7, allHighVoltageOnOff),
               std::runtime_error);
  EXPECT_THROW(NetDriverOffsets netDriverOffsets(-1, highVoltageNetOnOff, allHighVoltageOnOff),
               std::runtime_error);
  EXPECT_THROW(NetDriverOffsets netDriverOffsets(lowVoltageNetOnOff, highVoltageNetOnOff, -12), std::runtime_error);
}

TEST_F(TestNetDriverOffsets, Equals)
{
  NetDriverOffsets netDriverOffsets1(lowVoltageNetOnOff, highVoltageNetOnOff, allHighVoltageOnOff);
  NetDriverOffsets netDriverOffsets2(lowVoltageNetOnOff, highVoltageNetOnOff, allHighVoltageOnOff);

  EXPECT_TRUE(netDriverOffsets1 == netDriverOffsets2);
}
TEST_F(TestNetDriverOffsets, NotEquals)
{
  NetDriverOffsets netDriverOffsets1(11, highVoltageNetOnOff, allHighVoltageOnOff);
  NetDriverOffsets netDriverOffsets2(lowVoltageNetOnOff, 22, allHighVoltageOnOff);
  NetDriverOffsets netDriverOffsets3(lowVoltageNetOnOff, highVoltageNetOnOff, 33);

  EXPECT_FALSE(netDriverOffsets1 == netDriverOffsets2);
  EXPECT_FALSE(netDriverOffsets1 == netDriverOffsets3);
  EXPECT_FALSE(netDriverOffsets2 == netDriverOffsets3);
}

TEST_F(TestNetDriverOffsets, TestStream)
{
  NetDriverOffsets netDriverOffsets(lowVoltageNetOnOff, highVoltageNetOnOff, allHighVoltageOnOff);
  std::stringstream ss;
  ss << netDriverOffsets;
  EXPECT_EQ("NetDriverOffsets(lowVoltageNetOnOff: 1, highVoltageNetOnOff: 1, allHighVoltageOnOff: 1)", ss.str());
}