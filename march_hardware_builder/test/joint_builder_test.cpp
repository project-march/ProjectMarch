// Copyright 2019 Project March.
#include <string>

#include <gtest/gtest.h>
#include <ros/package.h>
#include <march_hardware_builder/hardware_config_exceptions.h>
#include <march_hardware_builder/hardware_builder.h>

class JointTest : public ::testing::Test
{
protected:
  std::string base_path;

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

  march::Joint createdJoint = HardwareBuilder::createJoint(jointConfig, "test_joint_hip");

  march::Encoder actualEncoder = march::Encoder(16, 22134, 43436, 24515, 0.05);
  march::IMotionCube actualIMotionCube = march::IMotionCube(2, actualEncoder, march::ActuationMode::unknown);
  march::TemperatureGES actualTemperatureGes = march::TemperatureGES(1, 2);
  march::Joint actualJoint(actualIMotionCube);
  actualJoint.setName("test_joint_hip");
  actualJoint.setAllowActuation(true);
  actualJoint.setTemperatureGes(actualTemperatureGes);

  ASSERT_EQ("test_joint_hip", actualJoint.getName());
  ASSERT_EQ(actualJoint, createdJoint);
}

TEST_F(JointTest, ValidNotActuated)
{
  std::string fullPath = this->fullPath("/joint_correct_not_actuated.yaml");
  YAML::Node jointConfig = YAML::LoadFile(fullPath);

  march::Joint createdJoint = HardwareBuilder::createJoint(jointConfig, "test_joint_hip");

  march::Encoder actualEncoder = march::Encoder(16, 22134, 43436, 24515, 0.05);
  march::IMotionCube actualIMotionCube = march::IMotionCube(2, actualEncoder, march::ActuationMode::unknown);
  march::TemperatureGES actualTemperatureGes = march::TemperatureGES(1, 2);
  march::Joint actualJoint(actualIMotionCube);
  actualJoint.setName("test_joint_hip");
  actualJoint.setAllowActuation(false);
  actualJoint.setTemperatureGes(actualTemperatureGes);

  march::Joint actualJointWrong(actualIMotionCube);

  actualJointWrong.setName("test_joint_hip");
  actualJointWrong.setAllowActuation(true);
  actualJointWrong.setTemperatureGes(actualTemperatureGes);
  ASSERT_EQ("test_joint_hip", actualJoint.getName());
  ASSERT_FALSE(actualJoint.canActuate());
  ASSERT_EQ(actualJoint, createdJoint);
  ASSERT_NE(actualJointWrong, createdJoint);
}

TEST_F(JointTest, ValidJointAnkle)
{
  std::string fullPath = this->fullPath("/joint_correct_2.yaml");
  YAML::Node jointConfig = YAML::LoadFile(fullPath);

  march::Joint createdJoint = HardwareBuilder::createJoint(jointConfig, "test_joint_ankle");

  march::Encoder actualEncoder = march::Encoder(20, 3, 40000, 5, 0.05);
  march::IMotionCube actualIMotionCube = march::IMotionCube(10, actualEncoder, march::ActuationMode::unknown);
  march::TemperatureGES actualTemperatureGes = march::TemperatureGES(10, 6);

  march::Joint actualJoint(actualIMotionCube);

  actualJoint.setName("test_joint_ankle");
  actualJoint.setAllowActuation(true);
  actualJoint.setTemperatureGes(actualTemperatureGes);

  ASSERT_EQ("test_joint_ankle", actualJoint.getName());
  ASSERT_EQ(actualJoint, createdJoint);
}

TEST_F(JointTest, NoActuate)
{
  std::string fullPath = this->fullPath("/joint_no_actuate.yaml");
  YAML::Node jointConfig = YAML::LoadFile(fullPath);

  ASSERT_THROW(HardwareBuilder::createJoint(jointConfig, "test_joint_no_actuate"), MissingKeyException);
}

TEST_F(JointTest, NoIMotionCube)
{
  std::string fullPath = this->fullPath("/joint_no_imotioncube.yaml");
  YAML::Node jointConfig = YAML::LoadFile(fullPath);

  ASSERT_THROW(HardwareBuilder::createJoint(jointConfig, "test_joint_no_imotioncube"), MissingKeyException);
}

TEST_F(JointTest, NoTemperatureGES)
{
  std::string fullPath = this->fullPath("/joint_no_temperature_ges.yaml");
  YAML::Node jointConfig = YAML::LoadFile(fullPath);

  ASSERT_NO_THROW(HardwareBuilder::createJoint(jointConfig, "test_joint_no_temperature_ges"));
}

TEST_F(JointTest, ValidActuationMode)
{
  std::string fullPath = this->fullPath("/joint_correct_position_mode.yaml");
  YAML::Node jointConfig = YAML::LoadFile(fullPath);

  march::Joint createdJoint = HardwareBuilder::createJoint(jointConfig, "test_joint_hip");

  march::Joint actualJoint(march::IMotionCube(1, march::Encoder(16, 22134, 43436, 24515, 0.05), march::ActuationMode::position));
  actualJoint.setName("test_joint_hip");

  ASSERT_EQ("test_joint_hip", actualJoint.getName());
  ASSERT_EQ(actualJoint, createdJoint);
}

TEST_F(JointDeathTest, EmptyJoint)
{
  std::string fullPath = this->fullPath("/joint_empty.yaml");
  YAML::Node jointConfig = YAML::LoadFile(fullPath);

  ASSERT_THROW(HardwareBuilder::createJoint(jointConfig, "test_joint_empty"), MissingKeyException);
}
