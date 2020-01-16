// Copyright 2018 Project March.

#include "gtest/gtest.h"
#include "march_hardware/TemperatureGES.h"

class TemperatureJointTest : public ::testing::Test
{
protected:
};

class TemperatureJointDeathTest : public TemperatureJointTest
{
};

TEST_F(TemperatureJointTest, SlaveIndexOne)
{
  march::TemperatureGES tempSens = march::TemperatureGES(1, 2);
  ASSERT_EQ(1, tempSens.getSlaveIndex());
}

TEST_F(TemperatureJointDeathTest, SlaveIndexZero)
{
  ASSERT_DEATH(march::TemperatureGES(0, 0), "Slave configuration error: slaveindex 0 can not be smaller than 1.");
}

TEST_F(TemperatureJointDeathTest, SlaveIndexMinusOne)
{
  ASSERT_DEATH(march::TemperatureGES(-1, 0), "Slave configuration error: slaveindex -1 can not be smaller than 1.");
}

TEST_F(TemperatureJointTest, ByteOffsetOne)
{
  ASSERT_NO_THROW(march::TemperatureGES(2, 1));
}

TEST_F(TemperatureJointTest, ByteOffsetZero)
{
  ASSERT_NO_THROW(march::TemperatureGES(2, 0));
}

TEST_F(TemperatureJointDeathTest, ByteOffsetMinusOne)
{
  ASSERT_DEATH(march::TemperatureGES(2, -1), "Slave configuration error: temperatureByteOffset -1 can not be "
                                             "negative.");
}

TEST_F(TemperatureJointTest, NoSlaveIndexConstructor)
{
  ASSERT_NO_THROW(march::TemperatureGES tempSens = march::TemperatureGES());
}
TEST_F(TemperatureJointTest, NoSlaveIndexConstructorGetIndex)
{
  march::TemperatureGES tempSens = march::TemperatureGES();
  ASSERT_EQ(-1, tempSens.getSlaveIndex());
}
