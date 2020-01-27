// Copyright 2019 Project March.
#include <string>
#include <gtest/gtest.h>
#include <ros/package.h>
#include <march_hardware_builder/hardware_config_exceptions.h>
#include <march_hardware_builder/hardware_builder.h>

class EncoderTest : public ::testing::Test
{
protected:
  std::string base_path;

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

  march::Encoder actualEncoder = march::Encoder(16, 22134, 43436, 24515, 0.05);
  march::Encoder createdEncoder = HardwareBuilder::createEncoder(encoderConfig);
  ASSERT_EQ(actualEncoder, createdEncoder);
}

TEST_F(EncoderTest, ValidEncoderAnkle)
{
  std::string fullPath = this->fullPath("/encoder_correct_2.yaml");
  YAML::Node encoderConfig = YAML::LoadFile(fullPath);

  march::Encoder actualEncoder = march::Encoder(12, 1086, 1490, 1301, 0.005);

  march::Encoder createdEncoder = HardwareBuilder::createEncoder(encoderConfig);
  ASSERT_EQ(actualEncoder, createdEncoder);
}

TEST_F(EncoderTest, NoResolution)
{
  std::string fullPath = this->fullPath("/encoder_no_resolution.yaml");
  YAML::Node encoderConfig = YAML::LoadFile(fullPath);

  ASSERT_THROW(HardwareBuilder::createEncoder(encoderConfig), MissingKeyException);
}

TEST_F(EncoderTest, NoMinPosition)
{
  std::string fullPath = this->fullPath("/encoder_no_min_position.yaml");
  YAML::Node encoderConfig = YAML::LoadFile(fullPath);

  ASSERT_THROW(HardwareBuilder::createEncoder(encoderConfig), MissingKeyException);
}

TEST_F(EncoderTest, NoMaxPosition)
{
  std::string fullPath = this->fullPath("/encoder_no_max_position.yaml");
  YAML::Node encoderConfig = YAML::LoadFile(fullPath);

  ASSERT_THROW(HardwareBuilder::createEncoder(encoderConfig), MissingKeyException);
}

TEST_F(EncoderTest, NoZeroPosition)
{
  std::string fullPath = this->fullPath("/encoder_no_zero_position.yaml");
  YAML::Node encoderConfig = YAML::LoadFile(fullPath);

  ASSERT_THROW(HardwareBuilder::createEncoder(encoderConfig), MissingKeyException);
}

TEST_F(EncoderTest, NoSafetyMargin)
{
  std::string fullPath = this->fullPath("/encoder_no_safety_margin.yaml");
  YAML::Node encoderConfig = YAML::LoadFile(fullPath);

  ASSERT_THROW(HardwareBuilder::createEncoder(encoderConfig), MissingKeyException);
}
