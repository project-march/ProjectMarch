// Copyright 2019 Project March.
#include <march_hardware_builder/hardware_builder.h>
#include <march_hardware_builder/hardware_config_exceptions.h>

#include <string>

#include <gtest/gtest.h>
#include <ros/package.h>

#include <march_hardware/encoder/incremental_encoder.h>

class IncrementalEncoderBuilderTest : public ::testing::Test {
protected:
    std::string base_path;

    void SetUp() override
    {
        this->base_path = ros::package::getPath("march_hardware_builder")
                              .append("/test/yaml/encoder");
    }

    YAML::Node loadTestYaml(const std::string& relative_path)
    {
        return YAML::LoadFile(this->base_path.append(relative_path));
    }
};

TEST_F(IncrementalEncoderBuilderTest, ValidIncrementalEncoder)
{
    YAML::Node config = this->loadTestYaml("/incremental_encoder_correct.yaml");

    march::IncrementalEncoder expected = march::IncrementalEncoder(12, 45.5);
    auto created = HardwareBuilder::createIncrementalEncoder(config);
    ASSERT_EQ(expected, *created);
}

TEST_F(IncrementalEncoderBuilderTest, NoConfig)
{
    YAML::Node config;
    ASSERT_EQ(nullptr, HardwareBuilder::createIncrementalEncoder(config[""]));
}

TEST_F(IncrementalEncoderBuilderTest, NoResolution)
{
    YAML::Node config
        = this->loadTestYaml("/incremental_encoder_no_resolution.yaml");

    ASSERT_THROW(
        HardwareBuilder::createIncrementalEncoder(config), MissingKeyException);
}

TEST_F(IncrementalEncoderBuilderTest, NoTransmission)
{
    YAML::Node config
        = this->loadTestYaml("/incremental_encoder_no_transmission.yaml");

    ASSERT_THROW(
        HardwareBuilder::createIncrementalEncoder(config), MissingKeyException);
}
