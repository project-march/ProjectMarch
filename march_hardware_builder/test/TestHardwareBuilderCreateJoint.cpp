// Copyright 2019 Project March.

#include "gtest/gtest.h"
#include "ros/ros.h"
#include <gmock/gmock.h>
#include <ros/package.h>
#include <march_hardware_builder/HardwareConfigExceptions.h>
#include <march_hardware_builder/HardwareBuilder.h>

using ::testing::Return;
using ::testing::AtLeast;

class JointTest : public ::testing::Test
{
protected:
  std::string base_path;
  HardwareBuilder hardwareBuilder;

  void SetUp() override
  {
    base_path = ros::package::getPath("march_hardware_builder").append("/test/yaml/joint");
  }

  std::string fullPath(const std::string& relativePath)
  {
    return this->base_path.append(relativePath);
  }
};

class JointDeathTest : public JointTest
{
};

TEST_F(JointTest, ValidJointHip)
{
  std::string fullPath = this->fullPath("/joint_correct_1.yaml");
  YAML::Node jointConfig = YAML::LoadFile(fullPath);

  march4cpp::Joint createdJoint = hardwareBuilder.createJoint(jointConfig, "test_joint_hip");

  march4cpp::Encoder actualEncoder = march4cpp::Encoder(16, 22134, 43436, 24515, 0.05);
  march4cpp::IMotionCube actualIMotionCube = march4cpp::IMotionCube(2, actualEncoder);
  march4cpp::TemperatureGES actualTemperatureGes = march4cpp::TemperatureGES(1, 2);
  march4cpp::Joint actualJoint = march4cpp::Joint("test_joint_hip", actualTemperatureGes, actualIMotionCube);

  ASSERT_EQ("test_joint_hip", actualJoint.getName());
  ASSERT_EQ(actualJoint, createdJoint);
}

TEST_F(JointTest, ValidJointAnkle)
{
  std::string fullPath = this->fullPath("/joint_correct_2.yaml");
  YAML::Node jointConfig = YAML::LoadFile(fullPath);

  march4cpp::Joint createdJoint = hardwareBuilder.createJoint(jointConfig, "test_joint_ankle");

  march4cpp::Encoder actualEncoder = march4cpp::Encoder(20, 3, 40000, 5, 0.05);
  march4cpp::IMotionCube actualIMotionCube = march4cpp::IMotionCube(10, actualEncoder);
  march4cpp::TemperatureGES actualTemperatureGes = march4cpp::TemperatureGES(10, 6);

  march4cpp::Joint actualJoint = march4cpp::Joint("test_joint_ankle", actualTemperatureGes, actualIMotionCube);
  ASSERT_EQ("test_joint_ankle", actualJoint.getName());
  ASSERT_EQ(actualJoint, createdJoint);
}

TEST_F(JointTest, NoIMotionCube)
{
  std::string fullPath = this->fullPath("/joint_no_imotioncube.yaml");
  YAML::Node jointConfig = YAML::LoadFile(fullPath);

  ASSERT_NO_THROW(hardwareBuilder.createJoint(jointConfig, "test_joint_no_imotioncube"));
}

TEST_F(JointTest, NoTemperatureGES)
{
  std::string fullPath = this->fullPath("/joint_no_temperature_ges.yaml");
  YAML::Node jointConfig = YAML::LoadFile(fullPath);

  ASSERT_NO_THROW(hardwareBuilder.createJoint(jointConfig, "test_joint_no_temperature_ges"));
}

TEST_F(JointDeathTest, EmptyJoint)
{
  std::string fullPath = this->fullPath("/joint_empty.yaml");
  YAML::Node jointConfig = YAML::LoadFile(fullPath);

  ASSERT_DEATH(hardwareBuilder.createJoint(jointConfig, "test_joint_empty_joint"),
               "Joint test_joint_empty_joint has no IMotionCube and no TemperatureGES. Please check its purpose.");
}
