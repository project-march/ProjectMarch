// Copyright 2018 Project March.

#include "gtest/gtest.h"
#include "march_hardware/TemperatureSensor.h"

class TestTemperatureJoint : public ::testing::Test
{
protected:
};

TEST_F(TestTemperatureJoint, SlaveIndexOne)
{
  march4cpp::TemperatureSensor tempSens = march4cpp::TemperatureSensor(1, 2);
  ASSERT_EQ(1, tempSens.getSlaveIndex());
}

TEST_F(TestTemperatureJoint, SlaveIndexZero)
{
  ASSERT_THROW(march4cpp::TemperatureSensor(0, 0), std::invalid_argument);
}

TEST_F(TestTemperatureJoint, SlaveIndexMinusOne)
{
  ASSERT_THROW(march4cpp::TemperatureSensor(-1, 0), std::invalid_argument);
}

TEST_F(TestTemperatureJoint, ByteOffsetOne)
{
  ASSERT_NO_THROW(march4cpp::TemperatureSensor(2, 1));
}

TEST_F(TestTemperatureJoint, ByteOffsetZero)
{
  ASSERT_NO_THROW(march4cpp::TemperatureSensor(2, 0));
}

TEST_F(TestTemperatureJoint, ByteOffsetMinusOne)
{
  ASSERT_THROW(march4cpp::TemperatureSensor(2, -1), std::invalid_argument);
}

TEST_F(TestTemperatureJoint, NoSlaveIndexConstructor)
{
  ASSERT_NO_THROW(march4cpp::TemperatureSensor tempSens = march4cpp::TemperatureSensor());
}
TEST_F(TestTemperatureJoint, NoSlaveIndexConstructorGetIndex)
{
  march4cpp::TemperatureSensor tempSens = march4cpp::TemperatureSensor();
  ASSERT_EQ(-1, tempSens.getSlaveIndex());
}
