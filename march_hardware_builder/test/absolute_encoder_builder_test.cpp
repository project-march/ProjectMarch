// Copyright 2019 Project March.
#include "march_hardware_builder/hardware_builder.h"
#include "march_hardware_builder/hardware_config_exceptions.h"

#include <string>

#include <gtest/gtest.h>
#include <ros/package.h>
#include <urdf/model.h>

#include <march_hardware/encoder/AbsoluteEncoder.h>

class TestAbsoluteEncoderBuilder : public ::testing::Test
{
protected:
  std::string base_path;
  urdf::JointSharedPtr joint;

  void SetUp() override
  {
    this->base_path = ros::package::getPath("march_hardware_builder").append("/test/yaml/encoder");
    this->joint = std::make_shared<urdf::Joint>();
    this->joint->limits = std::make_shared<urdf::JointLimits>();
    this->joint->safety = std::make_shared<urdf::JointSafety>();
  }

  YAML::Node loadTestYaml(const std::string& relative_path)
  {
    return YAML::LoadFile(this->base_path.append(relative_path));
  }
};

TEST_F(TestAbsoluteEncoderBuilder, ValidEncoderHip)
{
  YAML::Node config = this->loadTestYaml("/absolute_encoder_correct.yaml");
  this->joint->limits->lower = 0.0;
  this->joint->limits->upper = 2.0;
  this->joint->safety->soft_lower_limit = 0.1;
  this->joint->safety->soft_upper_limit = 1.9;

  march::AbsoluteEncoder expected =
      march::AbsoluteEncoder(16, 22134, 43436, this->joint->limits->lower, this->joint->limits->upper,
                             this->joint->safety->soft_lower_limit, this->joint->safety->soft_upper_limit);
  march::AbsoluteEncoder created = HardwareBuilder::createAbsoluteEncoder(config, this->joint);
  ASSERT_EQ(expected, created);
}

TEST_F(TestAbsoluteEncoderBuilder, NoResolution)
{
  YAML::Node config = this->loadTestYaml("/absolute_encoder_no_resolution.yaml");

  ASSERT_THROW(HardwareBuilder::createAbsoluteEncoder(config, this->joint), MissingKeyException);
}

TEST_F(TestAbsoluteEncoderBuilder, NoMinPosition)
{
  YAML::Node config = this->loadTestYaml("/absolute_encoder_no_min_position.yaml");

  ASSERT_THROW(HardwareBuilder::createAbsoluteEncoder(config, this->joint), MissingKeyException);
}

TEST_F(TestAbsoluteEncoderBuilder, NoMaxPosition)
{
  YAML::Node config = this->loadTestYaml("/absolute_encoder_no_max_position.yaml");

  ASSERT_THROW(HardwareBuilder::createAbsoluteEncoder(config, this->joint), MissingKeyException);
}
