// Copyright 2019 Project March.
#include <vector>
#include <gtest/gtest.h>
#include <ros/ros.h>
#include <gmock/gmock.h>
#include <ros/package.h>
#include <march_hardware_builder/hardware_config_exceptions.h>
#include <march_hardware_builder/hardware_builder.h>

using ::testing::AtLeast;
using ::testing::Return;

TEST(AllowedRobotTest, TestMarch4Creation)
{
  HardwareBuilder hardwareBuilder = HardwareBuilder(AllowedRobot::march4);
  ASSERT_NO_THROW(march::MarchRobot march4 = hardwareBuilder.createMarchRobot());
}

TEST(AllowedRobotTest, TestMarch3Creation)
{
  HardwareBuilder hardwareBuilder = HardwareBuilder(AllowedRobot::march3);
  ASSERT_NO_THROW(march::MarchRobot march3 = hardwareBuilder.createMarchRobot());
}

TEST(AllowedRobotTest, TestMarch3Values)
{
  march::MarchRobot march3 = HardwareBuilder(AllowedRobot::march3).createMarchRobot();

  march::Encoder RHJenc = march::Encoder(16, 22134, 43436, 24515, 0.05);
  march::Encoder LHJenc = march::Encoder(16, 9746, 31557, 11830, 0.05);

  march::Encoder RKJenc = march::Encoder(16, 18120, 39941, 19000, 0.05);
  march::Encoder LKJenc = march::Encoder(16, 21924, 43734, 22552, 0.05);

  march::Encoder RAJenc = march::Encoder(12, 1086, 1490, 1301, 0.005);
  march::Encoder LAJenc = march::Encoder(12, 631, 1022, 918, 0.005);

  march::IMotionCube LHJimc = march::IMotionCube(3, LHJenc);
  march::IMotionCube LKJimc = march::IMotionCube(5, LKJenc);
  march::IMotionCube LAJimc = march::IMotionCube(7, LAJenc);
  march::IMotionCube RHJimc = march::IMotionCube(8, RHJenc);
  march::IMotionCube RKJimc = march::IMotionCube(10, RKJenc);
  march::IMotionCube RAJimc = march::IMotionCube(12, RAJenc);

  march::Joint leftHip;
  leftHip.setName("left_hip");
  leftHip.setIMotionCube(LHJimc);
  leftHip.setActuationMode(march::ActuationMode("position"));

  march::Joint leftKnee;
  leftKnee.setName("left_knee");
  leftKnee.setIMotionCube(LKJimc);
  leftKnee.setActuationMode(march::ActuationMode("position"));

  march::Joint leftAnkle;
  leftAnkle.setName("left_ankle");
  leftAnkle.setIMotionCube(LAJimc);
  leftAnkle.setActuationMode(march::ActuationMode("position"));

  march::Joint rightHip;
  rightHip.setName("right_hip");
  rightHip.setIMotionCube(RHJimc);
  rightHip.setActuationMode(march::ActuationMode("position"));

  march::Joint rightKnee;
  rightKnee.setName("right_knee");
  rightKnee.setIMotionCube(RKJimc);
  rightKnee.setActuationMode(march::ActuationMode("position"));

  march::Joint rightAnkle;
  rightAnkle.setName("right_ankle");
  rightAnkle.setIMotionCube(RAJimc);
  rightAnkle.setActuationMode(march::ActuationMode("position"));

  std::vector<march::Joint> jointList;

  jointList.push_back(rightHip);
  jointList.push_back(leftHip);
  jointList.push_back(rightKnee);
  jointList.push_back(leftKnee);
  jointList.push_back(rightAnkle);
  jointList.push_back(leftAnkle);

  for (unsigned int i = 0; i < jointList.size(); i++)
  {
    jointList.at(i).setAllowActuation(true);
  }

  march::MarchRobot actualRobot = march::MarchRobot(jointList, "enp3s0", 4);

  EXPECT_EQ(actualRobot.getJoint("right_hip"), march3.getJoint("right_hip"));
  EXPECT_EQ(actualRobot.getJoint("left_hip"), march3.getJoint("left_hip"));
  EXPECT_EQ(actualRobot.getJoint("right_knee"), march3.getJoint("right_knee"));
  EXPECT_EQ(actualRobot.getJoint("left_knee"), march3.getJoint("left_knee"));
  EXPECT_EQ(actualRobot.getJoint("right_ankle"), march3.getJoint("right_ankle"));
  EXPECT_EQ(actualRobot.getJoint("left_ankle"), march3.getJoint("left_ankle"));
  ASSERT_EQ(actualRobot, march3);
}

TEST(AllowedRobotTest, TestTestRotationalSetupCreation)
{
  ASSERT_NO_THROW(HardwareBuilder(AllowedRobot::test_joint_rotational).createMarchRobot());
}

TEST(AllowedRobotTest, TestTestLinearSetupCreation)
{
  ASSERT_NO_THROW(HardwareBuilder(AllowedRobot::test_joint_linear).createMarchRobot());
}
