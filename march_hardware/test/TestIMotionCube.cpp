// Copyright 2018 Project March.

#include "gtest/gtest.h"
#include "march_hardware/IMotionCube.h"

class TestIMotionCube : public ::testing::Test
{
protected:
};

TEST_F(TestIMotionCube, SlaveIndexOne)
{
 march4cpp::IMotionCube imc = march4cpp::IMotionCube(1);
 ASSERT_EQ(1, imc.getSlaveIndex());
}

TEST_F(TestIMotionCube, SlaveIndexZero)
{
    ASSERT_THROW(march4cpp::IMotionCube(0), std::invalid_argument);
}

TEST_F(TestIMotionCube, SlaveIndexMinusOne)
{
    ASSERT_THROW(march4cpp::IMotionCube(-1), std::invalid_argument);
}

TEST_F(TestIMotionCube, NoSlaveIndexConstructor)
{
    ASSERT_NO_THROW(march4cpp::IMotionCube imc = march4cpp::IMotionCube());

}
TEST_F(TestIMotionCube, NoSlaveIndexConstructorGetIndex)
{
    march4cpp::IMotionCube imc = march4cpp::IMotionCube();
    ASSERT_EQ(-1, imc.getSlaveIndex());
}
