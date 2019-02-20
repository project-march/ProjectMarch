// Copyright 2018 Project March.

#include "gtest/gtest.h"
#include "march_hardware/TemperatureSensor.h"


class TestJoint : public ::testing::Test
{
protected:
};

TEST_F(TestJoint, SlaveIndexOne)
{
    march4cpp::TemperatureSensor tempSens = march4cpp::TemperatureSensor(1, 2);
    ASSERT_EQ(1, tempSens.getSlaveIndex());
}

TEST_F(TestJoint, SlaveIndexZero)
{
    ASSERT_THROW(march4cpp::TemperatureSensor(0, 0), std::invalid_argument);
}

TEST_F(TestJoint, SlaveIndexMinusOne)
{
    ASSERT_THROW(march4cpp::TemperatureSensor(-1, 0), std::invalid_argument);
}


TEST_F(TestJoint, ByteOffsetOne)
{
    ASSERT_NO_THROW(march4cpp::TemperatureSensor(2, 1));
}

TEST_F(TestJoint, ByteOffsetZero)
{
    ASSERT_NO_THROW(march4cpp::TemperatureSensor(2, 0));
}

TEST_F(TestJoint, ByteOffsetMinusOne)
{
    ASSERT_THROW(march4cpp::TemperatureSensor(2, -1), std::invalid_argument);
}

TEST_F(TestJoint, NoSlaveIndexConstructor)
{
    ASSERT_NO_THROW(march4cpp::TemperatureSensor tempSens = march4cpp::TemperatureSensor());

}
TEST_F(TestJoint, NoSlaveIndexConstructorGetIndex)
{
    march4cpp::TemperatureSensor tempSens = march4cpp::TemperatureSensor();
    ASSERT_EQ(-1, tempSens.getSlaveIndex());
}
