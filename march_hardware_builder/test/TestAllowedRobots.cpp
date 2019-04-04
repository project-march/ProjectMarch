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

  march4cpp::Encoder RHJenc = march4cpp::Encoder(16, 22134, 43436, 24515, 0.05);
  march4cpp::Encoder LHJenc = march4cpp::Encoder(16, 9746, 31557, 11830, 0.05);

  march4cpp::Encoder RKJenc = march4cpp::Encoder(16, 18120, 39941, 19000, 0.05);
  march4cpp::Encoder LKJenc = march4cpp::Encoder(16, 21924, 43734, 22552, 0.05);

  march4cpp::Encoder RAJenc = march4cpp::Encoder(12, 1086, 1490, 1301, 0.005);
  march4cpp::Encoder LAJenc = march4cpp::Encoder(12, 631, 1022, 918, 0.005);

  march4cpp::IMotionCube LHJimc = march4cpp::IMotionCube(3, LHJenc);
  march4cpp::IMotionCube LKJimc = march4cpp::IMotionCube(5, LKJenc);
  march4cpp::IMotionCube LAJimc = march4cpp::IMotionCube(7, LAJenc);
  march4cpp::IMotionCube RHJimc = march4cpp::IMotionCube(8, RHJenc);
  march4cpp::IMotionCube RKJimc = march4cpp::IMotionCube(10, RKJenc);
  march4cpp::IMotionCube RAJimc = march4cpp::IMotionCube(12, RAJenc);

  march4cpp::Joint leftHip = march4cpp::Joint("left_hip", LHJimc);
  march4cpp::Joint leftKnee = march4cpp::Joint("left_knee", LKJimc);
  march4cpp::Joint leftAnkle = march4cpp::Joint("left_ankle", LAJimc);
  march4cpp::Joint rightHip = march4cpp::Joint("right_hip", RHJimc);
  march4cpp::Joint rightKnee = march4cpp::Joint("right_knee", RKJimc);
  march4cpp::Joint rightAnkle = march4cpp::Joint("right_ankle", RAJimc);

  std::vector<march4cpp::Joint> jointList;
  jointList.push_back(rightHip);
  jointList.push_back(leftHip);
  jointList.push_back(rightKnee);
  jointList.push_back(leftKnee);
  jointList.push_back(rightAnkle);
  jointList.push_back(leftAnkle);

  march4cpp::MarchRobot actualRobot = march4cpp::MarchRobot(jointList, "enp3s0", 4);

  EXPECT_EQ(actualRobot.getJoint("right_hip"), march3.getJoint("right_hip"));
  EXPECT_EQ(actualRobot.getJoint("left_hip"), march3.getJoint("left_hip"));
  EXPECT_EQ(actualRobot.getJoint("right_knee"), march3.getJoint("right_knee"));
  EXPECT_EQ(actualRobot.getJoint("left_knee"), march3.getJoint("left_knee"));
  EXPECT_EQ(actualRobot.getJoint("right_ankle"), march3.getJoint("right_ankle"));
  EXPECT_EQ(actualRobot.getJoint("left_ankle"), march3.getJoint("left_ankle"));
  ASSERT_EQ(actualRobot, march3);
}

TEST_F(AllowedRobotTest, TestTestSetup)
{
  std::string fullPath = hardwareBuilder.getFilePathFromRobot(AllowedRobot::testsetup);
  YAML::Node testSetupConfig = YAML::LoadFile(fullPath);
  march4cpp::MarchRobot testSetup;
  ASSERT_NO_THROW(testSetup = hardwareBuilder.createMarchRobot(testSetupConfig));
}
