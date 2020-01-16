// Copyright 2019 Project March.
#include <string>

#include <gtest/gtest.h>
#include <ros/ros.h>
#include <gmock/gmock.h>
#include <ros/package.h>
#include <march_hardware_builder/hardware_config_exceptions.h>
#include <march_hardware_builder/hardware_builder.h>

using ::testing::AtLeast;
using ::testing::Return;

class IMotionCubeTest : public ::testing::Test
{
protected:
  std::string base_path;

  void SetUp() override
  {
    base_path = ros::package::getPath("march_hardware_builder").append("/test/yaml/imotioncube");
  }

  std::string fullPath(const std::string& relativePath)
  {
    return this->base_path.append(relativePath);
  }
};

TEST_F(IMotionCubeTest, ValidIMotionCubeHip)
{
  std::string fullPath = this->fullPath("/imotioncube_correct_1.yaml");
  YAML::Node iMotionCubeConfig = YAML::LoadFile(fullPath);

  march4cpp::IMotionCube createdIMotionCube = HardwareBuilder::createIMotionCube(iMotionCubeConfig);

  march4cpp::Encoder actualEncoder = march4cpp::Encoder(16, 22134, 43436, 24515, 0.05);
  march4cpp::IMotionCube actualIMotionCube = march4cpp::IMotionCube(2, actualEncoder);

  ASSERT_EQ(actualIMotionCube, createdIMotionCube);
}

TEST_F(IMotionCubeTest, ValidIMotionCubeAnkle)
{
  std::string fullPath = this->fullPath("/imotioncube_correct_2.yaml");
  YAML::Node iMotionCubeConfig = YAML::LoadFile(fullPath);

  march4cpp::IMotionCube createdIMotionCube = HardwareBuilder::createIMotionCube(iMotionCubeConfig);

  march4cpp::Encoder actualEncoder = march4cpp::Encoder(12, 1, 1000, 300, 0.01);
  march4cpp::IMotionCube actualIMotionCube = march4cpp::IMotionCube(10, actualEncoder);

  ASSERT_EQ(actualIMotionCube, createdIMotionCube);
}

TEST_F(IMotionCubeTest, IncorrectEncoder)
{
  std::string fullPath = this->fullPath("/imotioncube_incorrect_encoder.yaml");
  YAML::Node iMotionCubeConfig = YAML::LoadFile(fullPath);

  ASSERT_THROW(HardwareBuilder::createIMotionCube(iMotionCubeConfig), MissingKeyException);
}

TEST_F(IMotionCubeTest, NoEncoder)
{
  std::string fullPath = this->fullPath("/imotioncube_no_encoder.yaml");
  YAML::Node iMotionCubeConfig = YAML::LoadFile(fullPath);

  ASSERT_THROW(HardwareBuilder::createIMotionCube(iMotionCubeConfig), MissingKeyException);
}

TEST_F(IMotionCubeTest, NoSlaveIndex)
{
  std::string fullPath = this->fullPath("/imotioncube_no_slave_index.yaml");
  YAML::Node iMotionCubeConfig = YAML::LoadFile(fullPath);

  ASSERT_THROW(HardwareBuilder::createIMotionCube(iMotionCubeConfig), MissingKeyException);
}
