// Copyright 2019 Project March.

#include "gtest/gtest.h"
#include "ros/ros.h"
#include <gmock/gmock.h>
#include <ros/package.h>
#include <march_hardware_builder/HardwareConfigExceptions.h>
#include <march_hardware_builder/HardwareBuilder.h>

using ::testing::Return;
using ::testing::AtLeast;

class EncoderTest : public ::testing::Test
{
protected:
  std::string base_path;
  HardwareBuilder hardwareBuilder;

  void SetUp() override
  {
    base_path = ros::package::getPath("march_hardware_builder").append("/test/yaml/encoder");
  }

  std::string fullPath(const std::string& relativePath)
  {
    return this->base_path.append(relativePath);
  }
};

TEST_F(EncoderTest, ValidEncoderHip)
{
  std::string fullPath = this->fullPath("/encoder_correct_1.yaml");
  YAML::Node encoderConfig = YAML::LoadFile(fullPath);

  march4cpp::Encoder actualEncoder = march4cpp::Encoder(16, 22134, 43436, 24515, 0.05);
  march4cpp::Encoder createdEncoder = hardwareBuilder.createEncoder(encoderConfig);
  ASSERT_EQ(actualEncoder, createdEncoder);
}

TEST_F(EncoderTest, ValidEncoderAnkle)
{
  std::string fullPath = this->fullPath("/encoder_correct_2.yaml");
  YAML::Node encoderConfig = YAML::LoadFile(fullPath);

  march4cpp::Encoder actualEncoder = march4cpp::Encoder(12, 1086, 1490, 1301, 0.005);

  march4cpp::Encoder createdEncoder = hardwareBuilder.createEncoder(encoderConfig);
  ASSERT_EQ(actualEncoder, createdEncoder);
}

TEST_F(EncoderTest, NoResolution)
{
  std::string fullPath = this->fullPath("/encoder_no_resolution.yaml");
  YAML::Node encoderConfig = YAML::LoadFile(fullPath);

  ASSERT_THROW(hardwareBuilder.createEncoder(encoderConfig), MissingKeyException);
}

TEST_F(EncoderTest, NoMinPosition)
{
  std::string fullPath = this->fullPath("/encoder_no_min_position.yaml");
  YAML::Node encoderConfig = YAML::LoadFile(fullPath);

  ASSERT_THROW(hardwareBuilder.createEncoder(encoderConfig), MissingKeyException);
}

TEST_F(EncoderTest, NoMaxPosition)
{
  std::string fullPath = this->fullPath("/encoder_no_max_position.yaml");
  YAML::Node encoderConfig = YAML::LoadFile(fullPath);

  ASSERT_THROW(hardwareBuilder.createEncoder(encoderConfig), MissingKeyException);
}

TEST_F(EncoderTest, NoZeroPosition)
{
  std::string fullPath = this->fullPath("/encoder_no_zero_position.yaml");
  YAML::Node encoderConfig = YAML::LoadFile(fullPath);

  ASSERT_THROW(hardwareBuilder.createEncoder(encoderConfig), MissingKeyException);
}

TEST_F(EncoderTest, NoSafetyMargin)
{
  std::string fullPath = this->fullPath("/encoder_no_safety_margin.yaml");
  YAML::Node encoderConfig = YAML::LoadFile(fullPath);

  ASSERT_THROW(hardwareBuilder.createEncoder(encoderConfig), MissingKeyException);
}

class EncoderDeathTest : public EncoderTest
{
};

TEST_F(EncoderDeathTest, IncorrectResolution)
{
  std::string fullPath = this->fullPath("/encoder_incorrect_resolution.yaml");
  YAML::Node encoderConfig = YAML::LoadFile(fullPath);

  ASSERT_DEATH(hardwareBuilder.createEncoder(encoderConfig), "Encoder resolution of -1 is not within range \\(0, "
                                                             "32\\)");
}