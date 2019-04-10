// Copyright 2019 Project March.

#include "gtest/gtest.h"
#include "ros/ros.h"
#include <gmock/gmock.h>
#include <march_gait_generator/Pose.h>


using ::testing::Return;
using ::testing::AtLeast;

class PoseTest : public ::testing::Test
{
protected:
};


TEST_F(PoseTest, TestDummy)
{

    ASSERT_TRUE(true);
}
