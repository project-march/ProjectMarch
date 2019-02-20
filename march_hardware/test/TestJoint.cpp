// Copyright 2018 Project March.

#include "gtest/gtest.h"

#include "march_hardware/Joint.h"
#include "march_hardware/TemperatureSensor.h"
#include "march_hardware/IMotionCube.h"

class TestJoint : public ::testing::Test
{
protected:
};

TEST_F(TestJoint, TemperatureJoint)
{
  march4cpp::TemperatureSensor tempSens = march4cpp::TemperatureSensor(1, 0);
  march4cpp::Joint emptyJoint = march4cpp::Joint("empty-joint", tempSens);
}

TEST_F(TestJoint, SlaveIndexZero)
{
}

TEST_F(TestJoint, SlaveIndexMinusOne)
{
}
}