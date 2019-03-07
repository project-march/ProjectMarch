// Copyright 2018 Project March.

#include "gtest/gtest.h"
#include "march_hardware/IMotionCube.h"

class TestIMotionCube : public ::testing::Test
{
protected:
  march4cpp::Encoder encoder;

  void SetUp() override
  {
    encoder = march4cpp::Encoder();
  }
};

TEST_F(TestIMotionCube, SlaveIndexOne)
{
  march4cpp::IMotionCube imc = march4cpp::IMotionCube(1, encoder);
  ASSERT_EQ(1, imc.getSlaveIndex());
}

TEST_F(TestIMotionCube, SlaveIndexZero)
{
  ASSERT_THROW(march4cpp::IMotionCube(0, encoder), std::invalid_argument);
}

TEST_F(TestIMotionCube, SlaveIndexMinusOne)
{
  ASSERT_THROW(march4cpp::IMotionCube(-1, encoder), std::invalid_argument);
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
