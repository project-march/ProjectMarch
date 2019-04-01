// Copyright 2019 Project March.

#include "gtest/gtest.h"
#include "ros/ros.h"
#include <gmock/gmock.h>
#include <ros/package.h>
#include <march_hardware_builder/HardwareConfigExceptions.h>
#include <march_hardware_builder/HardwareBuilder.h>

using ::testing::Return;
using ::testing::AtLeast;

class IMotionCubeTest : public ::testing::Test
{
protected:
  std::string base_path;
  HardwareBuilder hardwareBuilder;

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

  march4cpp::IMotionCube createdIMotionCube = hardwareBuilder.createIMotionCube(iMotionCubeConfig);

  march4cpp::Encoder actualEncoder = march4cpp::Encoder(16, 22134, 43436, 24515, 0.05);
  march4cpp::IMotionCube actualIMotionCube = march4cpp::IMotionCube(2, actualEncoder);

  ASSERT_EQ(actualIMotionCube, createdIMotionCube);
}

TEST_F(IMotionCubeTest, ValidIMotionCubeAnkle)
{
  std::string fullPath = this->fullPath("/imotioncube_correct_2.yaml");
  YAML::Node iMotionCubeConfig = YAML::LoadFile(fullPath);

  march4cpp::IMotionCube createdIMotionCube = hardwareBuilder.createIMotionCube(iMotionCubeConfig);

  march4cpp::Encoder actualEncoder = march4cpp::Encoder(12, 1, 1000, 300, 0.01);
  march4cpp::IMotionCube actualIMotionCube = march4cpp::IMotionCube(10, actualEncoder);

  ASSERT_EQ(actualIMotionCube, createdIMotionCube);
}

TEST_F(IMotionCubeTest, IncorrectEncoder)
{
  std::string fullPath = this->fullPath("/imotioncube_incorrect_encoder.yaml");
  YAML::Node iMotionCubeConfig = YAML::LoadFile(fullPath);

  ASSERT_THROW(hardwareBuilder.createIMotionCube(iMotionCubeConfig), MissingKeyException);
}

TEST_F(IMotionCubeTest, NoEncoder)
{
  std::string fullPath = this->fullPath("/imotioncube_no_encoder.yaml");
  YAML::Node iMotionCubeConfig = YAML::LoadFile(fullPath);

  ASSERT_THROW(hardwareBuilder.createIMotionCube(iMotionCubeConfig), MissingKeyException);
}

TEST_F(IMotionCubeTest, NoSlaveIndex)
{
  std::string fullPath = this->fullPath("/imotioncube_no_slave_index.yaml");
  YAML::Node iMotionCubeConfig = YAML::LoadFile(fullPath);

  ASSERT_THROW(hardwareBuilder.createIMotionCube(iMotionCubeConfig), MissingKeyException);
}

class IMotionCubeDeathTest : public IMotionCubeTest
{
};

TEST_F(IMotionCubeDeathTest, IncorrectSlaveIndex)
{
  std::string fullPath = this->fullPath("/imotioncube_incorrect_slave_index.yaml");
  YAML::Node iMotionCubeConfig = YAML::LoadFile(fullPath);

  ASSERT_DEATH(hardwareBuilder.createIMotionCube(iMotionCubeConfig), "Slave configuration error: slaveindex -2 can not "
                                                                     "be smaller than 1.");
}