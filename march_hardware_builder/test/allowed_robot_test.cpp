// Copyright 2019 Project March.
#include "march_hardware_builder/hardware_builder.h"
#include "march_hardware_builder/hardware_config_exceptions.h"

#include <vector>

#include <gtest/gtest.h>
#include <ros/package.h>
#include <urdf/model.h>

TEST(AllowedRobotTest, TestMarch4Creation)
{
  urdf::Model urdf;
  urdf.initFile(ros::package::getPath("march_description").append("/urdf/march4.xacro"));
  HardwareBuilder hardwareBuilder = HardwareBuilder(AllowedRobot::march4, urdf);
  ASSERT_NO_THROW(march::MarchRobot march4 = hardwareBuilder.createMarchRobot());
}

TEST(AllowedRobotTest, TestMarch3Creation)
{
  urdf::Model urdf;
  urdf.initFile(ros::package::getPath("march_description").append("/urdf/march3.xacro"));
  HardwareBuilder hardwareBuilder = HardwareBuilder(AllowedRobot::march3, urdf);
  ASSERT_NO_THROW(march::MarchRobot march3 = hardwareBuilder.createMarchRobot());
}

TEST(AllowedRobotTest, TestMarch3Values)
{
  urdf::Model urdf;
  urdf.initFile(ros::package::getPath("march_description").append("/urdf/march3.xacro"));
  march::MarchRobot march3 = HardwareBuilder(AllowedRobot::march3, urdf).createMarchRobot();

  march::Encoder RHJenc = march::Encoder(16, 22134, 43436, 24515, 0.05);
  march::Encoder LHJenc = march::Encoder(16, 9746, 31557, 11830, 0.05);

  march::Encoder RKJenc = march::Encoder(16, 18120, 39941, 19000, 0.05);
  march::Encoder LKJenc = march::Encoder(16, 21924, 43734, 22552, 0.05);

  march::Encoder RAJenc = march::Encoder(12, 1086, 1490, 1301, 0.005);
  march::Encoder LAJenc = march::Encoder(12, 631, 1022, 918, 0.005);

  march::IMotionCube LHJimc = march::IMotionCube(3, LHJenc, march::ActuationMode::position);
  march::IMotionCube LKJimc = march::IMotionCube(5, LKJenc, march::ActuationMode::position);
  march::IMotionCube LAJimc = march::IMotionCube(7, LAJenc, march::ActuationMode::position);
  march::IMotionCube RHJimc = march::IMotionCube(8, RHJenc, march::ActuationMode::position);
  march::IMotionCube RKJimc = march::IMotionCube(10, RKJenc, march::ActuationMode::position);
  march::IMotionCube RAJimc = march::IMotionCube(12, RAJenc, march::ActuationMode::position);

  march::Joint leftHip(LHJimc);
  leftHip.setName("left_hip");

  march::Joint leftKnee(LKJimc);
  leftKnee.setName("left_knee");

  march::Joint leftAnkle(LAJimc);
  leftAnkle.setName("left_ankle");

  march::Joint rightHip(RHJimc);
  rightHip.setName("right_hip");

  march::Joint rightKnee(RKJimc);
  rightKnee.setName("right_knee");

  march::Joint rightAnkle(RAJimc);
  rightAnkle.setName("right_ankle");

  std::vector<march::Joint> jointList;

  jointList.push_back(rightHip);
  jointList.push_back(leftHip);
  jointList.push_back(rightKnee);
  jointList.push_back(leftKnee);
  jointList.push_back(rightAnkle);
  jointList.push_back(leftAnkle);

  for (march::Joint& joint : jointList)
  {
    joint.setAllowActuation(true);
  }

  march::MarchRobot actualRobot = march::MarchRobot(jointList, urdf, "enp3s0", 4);

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
  urdf::Model urdf;
  urdf.initFile(ros::package::getPath("march_description").append("/urdf/test_joint_rotational.xacro"));
  ASSERT_NO_THROW(HardwareBuilder(AllowedRobot::test_joint_rotational, urdf).createMarchRobot());
}

TEST(AllowedRobotTest, TestTestLinearSetupCreation)
{
  urdf::Model urdf;
  urdf.initFile(ros::package::getPath("march_description").append("/urdf/test_joint_linear.xacro"));
  ASSERT_NO_THROW(HardwareBuilder(AllowedRobot::test_joint_linear, urdf).createMarchRobot());
}
