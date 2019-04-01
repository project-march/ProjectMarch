// Copyright 2019 Project March.

#include "gtest/gtest.h"
#include "ros/ros.h"
#include <gmock/gmock.h>

#include <march_hardware_builder/HardwareConfigExceptions.h>
#include <march_hardware_builder/HardwareBuilder.h>

using ::testing::Return;
using ::testing::AtLeast;

class EncoderTest : public ::testing::Test
{
protected:
  HardwareBuilder hardwareBuilder;

  void SetUp() override
  {
  }
};

TEST_F(EncoderTest, ValidEncoder)
{
  YAML::Node encoderConfig =
          YAML::LoadFile("/home/projectmarch/Documents/march-iv/march_ws/src/hardware-interface/march_hardware_builder/"
                     "test/yaml/encoder_correct.yaml");


  march4cpp::Encoder actualEncoder = march4cpp::Encoder(16, 22134, 43436, 24515, 0.05);
  march4cpp::Encoder createdEncoder = hardwareBuilder.createEncoder(encoderConfig);
  ASSERT_EQ(actualEncoder, createdEncoder);

}