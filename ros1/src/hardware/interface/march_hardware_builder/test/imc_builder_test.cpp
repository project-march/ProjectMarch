// Copyright 2019 Project March.
#include "march_hardware_builder/hardware_builder.h"
#include "march_hardware_builder/hardware_config_exceptions.h"

#include <string>

#include <gtest/gtest.h>
#include <ros/package.h>
#include <urdf/model.h>

#include <march_hardware/encoder/absolute_encoder.h>
#include <march_hardware/encoder/incremental_encoder.h>
#include <march_hardware/motor_controller/imotioncube/imotioncube.h>

class IMotionCubeBuilderTest : public ::testing::Test
{
protected:
  std::string base_path;
  urdf::JointSharedPtr joint;
  march::PdoInterfacePtr pdo_interface;
  march::SdoInterfacePtr sdo_interface;

  void SetUp() override
  {
    this->base_path = ros::package::getPath("march_hardware_builder").append("/test/yaml/imotioncube");
    this->joint = std::make_shared<urdf::Joint>();
    this->joint->limits = std::make_shared<urdf::JointLimits>();
    this->joint->safety = std::make_shared<urdf::JointSafety>();
    this->pdo_interface = march::PdoInterfaceImpl::create();
    this->sdo_interface = march::SdoInterfaceImpl::create();
  }

  YAML::Node loadTestYaml(const std::string& relative_path)
  {
    return YAML::LoadFile(this->base_path.append(relative_path));
  }
};

TEST_F(IMotionCubeBuilderTest, ValidIMotionCubeHip)
{
  YAML::Node config = this->loadTestYaml("/imotioncube_correct.yaml");
  this->joint->limits->lower = 0.0;
  this->joint->limits->upper = 2.0;
  this->joint->safety->soft_lower_limit = 0.1;
  this->joint->safety->soft_upper_limit = 1.9;

  auto created = HardwareBuilder::createIMotionCube(config, march::ActuationMode::unknown, this->joint,
                                                    this->pdo_interface, this->sdo_interface);

  auto absolute_encoder = std::make_unique<march::AbsoluteEncoder>(
      16, 22134, 43436, this->joint->limits->lower, this->joint->limits->upper, this->joint->safety->soft_lower_limit,
      this->joint->safety->soft_upper_limit);
  auto incremental_encoder = std::make_unique<march::IncrementalEncoder>(12, 101.0);
  march::IMotionCube expected(march::Slave(2, this->pdo_interface, this->sdo_interface), std::move(absolute_encoder),
                              std::move(incremental_encoder), march::ActuationMode::unknown);

  ASSERT_EQ(expected, *created);
}

TEST_F(IMotionCubeBuilderTest, NoConfig)
{
  YAML::Node config;
  ASSERT_EQ(nullptr, HardwareBuilder::createIMotionCube(config["imotioncube"], march::ActuationMode::unknown,
                                                        this->joint, this->pdo_interface, this->sdo_interface));
}

TEST_F(IMotionCubeBuilderTest, NoUrdfJoint)
{
  YAML::Node config = this->loadTestYaml("/imotioncube_correct.yaml");
  ASSERT_EQ(nullptr, HardwareBuilder::createIMotionCube(config, march::ActuationMode::unknown, nullptr,
                                                        this->pdo_interface, this->sdo_interface));
}

TEST_F(IMotionCubeBuilderTest, NoAbsoluteEncoder)
{
  YAML::Node config = this->loadTestYaml("/imotioncube_no_absolute_encoder.yaml");

  ASSERT_THROW(HardwareBuilder::createIMotionCube(config, march::ActuationMode::unknown, this->joint,
                                                  this->pdo_interface, this->sdo_interface),
               MissingKeyException);
}

TEST_F(IMotionCubeBuilderTest, NoIncrementalEncoder)
{
  YAML::Node config = this->loadTestYaml("/imotioncube_no_incremental_encoder.yaml");

  ASSERT_THROW(HardwareBuilder::createIMotionCube(config, march::ActuationMode::unknown, this->joint,
                                                  this->pdo_interface, this->sdo_interface),
               MissingKeyException);
}

TEST_F(IMotionCubeBuilderTest, NoSlaveIndex)
{
  YAML::Node config = this->loadTestYaml("/imotioncube_no_slave_index.yaml");

  ASSERT_THROW(HardwareBuilder::createIMotionCube(config, march::ActuationMode::unknown, this->joint,
                                                  this->pdo_interface, this->sdo_interface),
               MissingKeyException);
}
