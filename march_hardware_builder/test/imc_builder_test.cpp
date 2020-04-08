// Copyright 2019 Project March.
#include "march_hardware_builder/hardware_builder.h"
#include "march_hardware_builder/hardware_config_exceptions.h"

#include <string>

#include <gtest/gtest.h>
#include <ros/package.h>
#include <urdf/model.h>

#include <march_hardware/encoder/AbsoluteEncoder.h>
#include <march_hardware/encoder/IncrementalEncoder.h>
#include <march_hardware/IMotionCube.h>

class IMotionCubeTest : public ::testing::Test
{
protected:
  std::string base_path;
  urdf::JointSharedPtr joint;

  void SetUp() override
  {
    this->base_path = ros::package::getPath("march_hardware_builder").append("/test/yaml/imotioncube");
    this->joint = std::make_shared<urdf::Joint>();
    this->joint->limits = std::make_shared<urdf::JointLimits>();
    this->joint->safety = std::make_shared<urdf::JointSafety>();
  }

  YAML::Node loadTestYaml(const std::string& relative_path)
  {
    return YAML::LoadFile(this->base_path.append(relative_path));
  }
};

TEST_F(IMotionCubeTest, ValidIMotionCubeHip)
{
  YAML::Node config = this->loadTestYaml("/imotioncube_correct.yaml");
  this->joint->limits->lower = 0.0;
  this->joint->limits->upper = 2.0;
  this->joint->safety->soft_lower_limit = 0.1;
  this->joint->safety->soft_upper_limit = 1.9;

  std::string sw_stream_empty_;

  auto created = HardwareBuilder::createIMotionCube(config, march::ActuationMode::unknown, this->joint);

  auto absolute_encoder = std::make_unique<march::AbsoluteEncoder>(
      16, 22134, 43436, this->joint->limits->lower, this->joint->limits->upper, this->joint->safety->soft_lower_limit,
      this->joint->safety->soft_upper_limit);
  auto incremental_encoder = std::make_unique<march::IncrementalEncoder>(12, 101.0);
  march::IMotionCube expected(2, std::move(absolute_encoder), std::move(incremental_encoder), sw_stream_empty_,
                              march::ActuationMode::unknown);

  ASSERT_EQ(expected, *created);
}

TEST_F(IMotionCubeTest, NoConfig)
{
  YAML::Node config;
  ASSERT_EQ(nullptr,
            HardwareBuilder::createIMotionCube(config["imotioncube"], march::ActuationMode::unknown, this->joint));
}

TEST_F(IMotionCubeTest, NoAbsoluteEncoder)
{
  YAML::Node iMotionCubeConfig = this->loadTestYaml("/imotioncube_no_absolute_encoder.yaml");

  ASSERT_THROW(HardwareBuilder::createIMotionCube(iMotionCubeConfig, march::ActuationMode::unknown, this->joint),
               MissingKeyException);
}

TEST_F(IMotionCubeTest, NoIncrementalEncoder)
{
  YAML::Node iMotionCubeConfig = this->loadTestYaml("/imotioncube_no_incremental_encoder.yaml");

  ASSERT_THROW(HardwareBuilder::createIMotionCube(iMotionCubeConfig, march::ActuationMode::unknown, this->joint),
               MissingKeyException);
}

TEST_F(IMotionCubeTest, NoSlaveIndex)
{
  YAML::Node iMotionCubeConfig = this->loadTestYaml("/imotioncube_no_slave_index.yaml");

  ASSERT_THROW(HardwareBuilder::createIMotionCube(iMotionCubeConfig, march::ActuationMode::unknown, this->joint),
               MissingKeyException);
}
