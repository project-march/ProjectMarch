// Copyright 2019 Project March.
#include "march_hardware_builder/hardware_builder.h"
#include "march_hardware_builder/hardware_config_exceptions.h"

#include <string>

#include <gtest/gtest.h>
#include <ros/package.h>
#include <urdf/model.h>

class EncoderTest : public ::testing::Test
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

TEST_F(EncoderTest, ValidEncoderHip)
{
  YAML::Node config = this->loadTestYaml("/encoder_correct.yaml");
  this->joint->limits->lower = 0.0;
  this->joint->limits->upper = 2.0;
  this->joint->safety->soft_lower_limit = 0.1;
  this->joint->safety->soft_upper_limit = 1.9;

  march::Encoder expected =
      march::Encoder(16, 22134, 43436, this->joint->limits->lower, this->joint->limits->upper,
                     this->joint->safety->soft_lower_limit, this->joint->safety->soft_upper_limit);
  march::Encoder created = HardwareBuilder::createEncoder(config, this->joint);
  ASSERT_EQ(expected, created);
}

TEST_F(EncoderTest, NoResolution)
{
  YAML::Node config = this->loadTestYaml("/encoder_no_resolution.yaml");

  ASSERT_THROW(HardwareBuilder::createEncoder(config, this->joint), MissingKeyException);
}

TEST_F(EncoderTest, NoMinPosition)
{
  YAML::Node config = this->loadTestYaml("/encoder_no_min_position.yaml");

  ASSERT_THROW(HardwareBuilder::createEncoder(config, this->joint), MissingKeyException);
}

TEST_F(EncoderTest, NoMaxPosition)
{
  YAML::Node config = this->loadTestYaml("/encoder_no_max_position.yaml");

  ASSERT_THROW(HardwareBuilder::createEncoder(config, this->joint), MissingKeyException);
}
