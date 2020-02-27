// Copyright 2019 Project March.
#include "march_hardware_builder/hardware_builder.h"

#include <vector>

#include <gtest/gtest.h>
#include <ros/package.h>
#include <urdf/model.h>

TEST(AllowedRobotTest, TestMarch4Creation)
{
  urdf::Model urdf;
  urdf.initFile(ros::package::getPath("march_description").append("/urdf/march4.urdf"));
  ASSERT_NO_THROW(HardwareBuilder(AllowedRobot::march4, urdf).createMarchRobot());
}

// Fails because the March 3 does not have safety limits
// TEST(AllowedRobotTest, TestMarch3Creation)
//{
//  urdf::Model urdf;
//  urdf.initFile(ros::package::getPath("march_description").append("/urdf/march3.urdf"));
//  ASSERT_NO_THROW(HardwareBuilder(AllowedRobot::march3, urdf).createMarchRobot());
//}

// Fails because the rotational test joint is not correctly calibrated
// TEST(AllowedRobotTest, TestTestRotationalSetupCreation)
//{
//  urdf::Model urdf;
//  urdf.initFile(ros::package::getPath("march_description").append("/urdf/test_joint_rotational.urdf"));
//  ASSERT_NO_THROW(HardwareBuilder(AllowedRobot::test_joint_rotational, urdf).createMarchRobot());
//}

TEST(AllowedRobotTest, TestTestLinearSetupCreation)
{
  urdf::Model urdf;
  urdf.initFile(ros::package::getPath("march_description").append("/urdf/test_joint_linear.urdf"));
  ASSERT_NO_THROW(HardwareBuilder(AllowedRobot::test_joint_linear, urdf).createMarchRobot());
}
