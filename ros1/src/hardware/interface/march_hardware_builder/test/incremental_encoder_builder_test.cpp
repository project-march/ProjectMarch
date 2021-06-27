// Copyright 2019 Project March.
#include <march_hardware_builder/hardware_builder.h>
#include <march_hardware_builder/hardware_config_exceptions.h>

#include <string>

#include <gtest/gtest.h>
#include <ros/package.h>

#include <march_hardware/encoder/incremental_encoder.h>
#include <march_hardware/motor_controller/motor_controller_type.h>

class IncrementalEncoderBuilderTest : public ::testing::Test {
protected:
    std::string base_path;
    const march::MotorControllerType motor_controller_type
        = march::MotorControllerType::IMotionCube;

    void SetUp() override
    {
        this->base_path = ros::package::getPath("march_hardware_builder")
                              .append(/*__s=*/"/test/yaml/encoder");
    }

    YAML::Node loadTestYaml(const std::string& relative_path)
    {
        return YAML::LoadFile(this->base_path.append(relative_path));
    }
};

TEST_F(IncrementalEncoderBuilderTest, ValidIncrementalEncoder)
{
    YAML::Node config = this->loadTestYaml("/incremental_encoder_correct.yaml");

    march::IncrementalEncoder expected = march::IncrementalEncoder(
        /*resolution=*/12, motor_controller_type, /*transmission=*/45.5);
    auto created = HardwareBuilder::createIncrementalEncoder(
        config, motor_controller_type);
    ASSERT_EQ(expected, *created);
}

TEST_F(IncrementalEncoderBuilderTest, NoConfig)
{
    YAML::Node config;
    ASSERT_EQ(nullptr,
        HardwareBuilder::createIncrementalEncoder(
            config[""], motor_controller_type));
}

TEST_F(IncrementalEncoderBuilderTest, NoResolution)
{
    YAML::Node config
        = this->loadTestYaml("/incremental_encoder_no_resolution.yaml");

    ASSERT_THROW(HardwareBuilder::createIncrementalEncoder(
                     config, motor_controller_type),
        MissingKeyException);
}

TEST_F(IncrementalEncoderBuilderTest, NoTransmission)
{
    YAML::Node config
        = this->loadTestYaml("/incremental_encoder_no_transmission.yaml");

    ASSERT_THROW(HardwareBuilder::createIncrementalEncoder(
                     config, motor_controller_type),
        MissingKeyException);
}
