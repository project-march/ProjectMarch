// Copyright 2019 Project March.

#include "gtest/gtest.h"
#include "ros/ros.h"
#include <gmock/gmock.h>
#include <ros/package.h>
#include <march_hardware_builder/HardwareConfigExceptions.h>
#include <march_hardware_builder/HardwareBuilder.h>

using ::testing::AtLeast;
using ::testing::Return;

class AllowedRobotTest : public ::testing::Test
{
protected:
};

TEST_F(AllowedRobotTest, TestMarch3Creation)
{
  HardwareBuilder hardwareBuilder = HardwareBuilder(AllowedRobot::march3);
  ASSERT_NO_THROW(march4cpp::MarchRobot march3 = hardwareBuilder.createMarchRobot());
}

TEST_F(AllowedRobotTest, TestMarch3Values)
{
  march4cpp::MarchRobot march3 = HardwareBuilder(AllowedRobot::march3).createMarchRobot();

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

  march4cpp::Joint leftHip;
  leftHip.setName("left_hip");
  leftHip.setIMotionCube(LHJimc);
  leftHip.setActuationMode(march4cpp::ActuationMode("position"));

  march4cpp::Joint leftKnee;
  leftKnee.setName("left_knee");
  leftKnee.setIMotionCube(LKJimc);
  leftKnee.setActuationMode(march4cpp::ActuationMode("position"));

  march4cpp::Joint leftAnkle;
  leftAnkle.setName("left_ankle");
  leftAnkle.setIMotionCube(LAJimc);
  leftAnkle.setActuationMode(march4cpp::ActuationMode("position"));

  march4cpp::Joint rightHip;
  rightHip.setName("right_hip");
  rightHip.setIMotionCube(RHJimc);
  rightHip.setActuationMode(march4cpp::ActuationMode("position"));

  march4cpp::Joint rightKnee;
  rightKnee.setName("right_knee");
  rightKnee.setIMotionCube(RKJimc);
  rightKnee.setActuationMode(march4cpp::ActuationMode("position"));

  march4cpp::Joint rightAnkle;
  rightAnkle.setName("right_ankle");
  rightAnkle.setIMotionCube(RAJimc);
  rightAnkle.setActuationMode(march4cpp::ActuationMode("position"));

  std::vector<march4cpp::Joint> jointList;

  jointList.push_back(rightHip);
  jointList.push_back(leftHip);
  jointList.push_back(rightKnee);
  jointList.push_back(leftKnee);
  jointList.push_back(rightAnkle);
  jointList.push_back(leftAnkle);

  for (int i = 0; i < jointList.size(); i++)
  {
    jointList.at(i).setAllowActuation(true);
  }

  march4cpp::MarchRobot actualRobot = march4cpp::MarchRobot(jointList, "enp3s0", 4);

  EXPECT_EQ(actualRobot.getJoint("right_hip"), march3.getJoint("right_hip"));
  EXPECT_EQ(actualRobot.getJoint("left_hip"), march3.getJoint("left_hip"));
  EXPECT_EQ(actualRobot.getJoint("right_knee"), march3.getJoint("right_knee"));
  EXPECT_EQ(actualRobot.getJoint("left_knee"), march3.getJoint("left_knee"));
  EXPECT_EQ(actualRobot.getJoint("right_ankle"), march3.getJoint("right_ankle"));
  EXPECT_EQ(actualRobot.getJoint("left_ankle"), march3.getJoint("left_ankle"));
  ASSERT_EQ(actualRobot, march3);
}

TEST_F(AllowedRobotTest, TestTestSetupCreation)
{
  ASSERT_NO_THROW(march4cpp::MarchRobot testSetup = HardwareBuilder(AllowedRobot::testsetup).createMarchRobot());
}
