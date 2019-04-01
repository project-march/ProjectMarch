// Copyright 2019 Project March.

#include "gtest/gtest.h"
#include "ros/ros.h"
#include <gmock/gmock.h>
#include <ros/package.h>
#include <march_hardware_builder/HardwareConfigExceptions.h>
#include <march_hardware_builder/HardwareBuilder.h>

using ::testing::Return;
using ::testing::AtLeast;

class AllowedRobotTest : public ::testing::Test
{
protected:
  HardwareBuilder hardwareBuilder;
};

TEST_F(AllowedRobotTest, TestMarch3)
{
  std::string fullPath = hardwareBuilder.getFilePathFromRobot(AllowedRobot::march3);
  YAML::Node march3Config = YAML::LoadFile(fullPath);
  march4cpp::MarchRobot march3;
  ASSERT_NO_THROW(march3 = hardwareBuilder.createMarchRobot(march3Config));
  ROS_INFO_STREAM(march3);
}

TEST_F(AllowedRobotTest, TestTestSetup)
{
    std::string fullPath = hardwareBuilder.getFilePathFromRobot(AllowedRobot::testsetup);
    YAML::Node testSetupConfig = YAML::LoadFile(fullPath);
    march4cpp::MarchRobot testSetup;
    ASSERT_NO_THROW(testSetup = hardwareBuilder.createMarchRobot(testSetupConfig));
}
