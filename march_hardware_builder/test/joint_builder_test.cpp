// Copyright 2019 Project March.
#include "march_hardware_builder/hardware_builder.h"
#include "march_hardware_builder/hardware_config_exceptions.h"

#include <string>

#include <gtest/gtest.h>
#include <ros/package.h>
#include <urdf/model.h>

class JointTest : public ::testing::Test
{
protected:
  std::string base_path;
  urdf::JointSharedPtr joint;

  void SetUp() override
  {
    this->base_path = ros::package::getPath("march_hardware_builder").append("/test/yaml/joint");
    this->joint = std::make_shared<urdf::Joint>();
    this->joint->limits = std::make_shared<urdf::JointLimits>();
    this->joint->safety = std::make_shared<urdf::JointSafety>();
  }

  YAML::Node loadTestYaml(const std::string& relative_path)
  {
    return YAML::LoadFile(this->base_path.append(relative_path));
  }
};

TEST_F(JointTest, ValidJointHip)
{
  YAML::Node config = this->loadTestYaml("/joint_correct.yaml");
  this->joint->limits->lower = 0.0;
  this->joint->limits->upper = 2.0;
  this->joint->safety->soft_lower_limit = 0.1;
  this->joint->safety->soft_upper_limit = 1.9;

  march::Joint created = HardwareBuilder::createJoint(config, "test_joint_hip", this->joint);

  march::Encoder encoder = march::Encoder(16, 22134, 43436, this->joint->limits->lower, this->joint->limits->upper,
                                          this->joint->safety->soft_lower_limit, this->joint->safety->soft_upper_limit);
  march::IMotionCube imc = march::IMotionCube(2, encoder, march::ActuationMode::unknown);
  march::TemperatureGES ges = march::TemperatureGES(1, 2);
  march::Joint expected(imc);
  expected.setName("test_joint_hip");
  expected.setAllowActuation(true);
  expected.setTemperatureGes(ges);

  ASSERT_EQ("test_joint_hip", expected.getName());
  ASSERT_EQ(expected, created);
}

TEST_F(JointTest, ValidNotActuated)
{
  YAML::Node config = this->loadTestYaml("/joint_correct_not_actuated.yaml");
  this->joint->limits->lower = 0.0;
  this->joint->limits->upper = 2.0;
  this->joint->safety->soft_lower_limit = 0.1;
  this->joint->safety->soft_upper_limit = 1.9;

  march::Joint created = HardwareBuilder::createJoint(config, "test_joint_hip", this->joint);

  march::Encoder encoder = march::Encoder(16, 22134, 43436, this->joint->limits->lower, this->joint->limits->upper,
                                          this->joint->safety->soft_lower_limit, this->joint->safety->soft_upper_limit);
  march::IMotionCube imc = march::IMotionCube(2, encoder, march::ActuationMode::unknown);
  march::TemperatureGES ges = march::TemperatureGES(1, 2);
  march::Joint expected(imc);
  expected.setName("test_joint_hip");
  expected.setAllowActuation(false);
  expected.setTemperatureGes(ges);

  ASSERT_EQ("test_joint_hip", expected.getName());
  ASSERT_FALSE(expected.canActuate());
  ASSERT_EQ(expected, created);
}

TEST_F(JointTest, NoActuate)
{
  YAML::Node config = this->loadTestYaml("/joint_no_actuate.yaml");

  ASSERT_THROW(HardwareBuilder::createJoint(config, "test_joint_no_actuate", this->joint), MissingKeyException);
}

TEST_F(JointTest, NoIMotionCube)
{
  YAML::Node config = this->loadTestYaml("/joint_no_imotioncube.yaml");

  ASSERT_THROW(HardwareBuilder::createJoint(config, "test_joint_no_imotioncube", this->joint), MissingKeyException);
}

TEST_F(JointTest, NoTemperatureGES)
{
  YAML::Node config = this->loadTestYaml("/joint_no_temperature_ges.yaml");
  this->joint->limits->lower = 0.0;
  this->joint->limits->upper = 0.24;
  this->joint->safety->soft_lower_limit = 0.1;
  this->joint->safety->soft_upper_limit = 0.15;

  ASSERT_NO_THROW(HardwareBuilder::createJoint(config, "test_joint_no_temperature_ges", this->joint));
}

TEST_F(JointTest, ValidActuationMode)
{
  YAML::Node config = this->loadTestYaml("/joint_correct_position_mode.yaml");
  this->joint->limits->lower = 0.0;
  this->joint->limits->upper = 2.0;
  this->joint->safety->soft_lower_limit = 0.1;
  this->joint->safety->soft_upper_limit = 1.9;

  march::Joint created = HardwareBuilder::createJoint(config, "test_joint_hip", this->joint);

  march::Joint expected(
      march::IMotionCube(1,
                         march::Encoder(16, 22134, 43436, this->joint->limits->lower, this->joint->limits->upper,
                                        this->joint->safety->soft_lower_limit, this->joint->safety->soft_upper_limit),
                         march::ActuationMode::position));
  expected.setName("test_joint_hip");

  ASSERT_EQ("test_joint_hip", expected.getName());
  ASSERT_EQ(expected, created);
}

TEST_F(JointTest, EmptyJoint)
{
  YAML::Node config;
  ASSERT_THROW(HardwareBuilder::createJoint(config, "test_joint_empty", this->joint), MissingKeyException);
}
