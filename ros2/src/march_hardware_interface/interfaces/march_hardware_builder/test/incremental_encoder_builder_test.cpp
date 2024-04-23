#ifndef __clang_analyzer__
// Copyright 2019 Project March.
#include <march_hardware_builder/hardware_builder.h>
#include <march_hardware_builder/hardware_config_exceptions.h>

#include <string>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <gtest/gtest.h>

#include <march_hardware/encoder/incremental_encoder.h>
#include <march_hardware/motor_controller/motor_controller_type.h>

class IncrementalEncoderBuilderTest : public ::testing::Test {
protected:
    std::string base_path;
    const march::MotorControllerType motor_controller_type = march::MotorControllerType::IMotionCube;

    void SetUp() override
    {
        this->base_path = ament_index_cpp::get_package_share_directory("march_hardware_builder")
                              .append("/robots/test_yamls/encoder");
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
        /*counts_per_rotation=*/(size_t)1 << 12, motor_controller_type,
        /*transmission=*/45.5);
    auto created = HardwareBuilder::createIncrementalEncoder(config, motor_controller_type);
    ASSERT_EQ(expected, *created);
}

TEST_F(IncrementalEncoderBuilderTest, NoConfig)
{
    YAML::Node config;
    ASSERT_EQ(nullptr, HardwareBuilder::createIncrementalEncoder(config[""], motor_controller_type));
}

TEST_F(IncrementalEncoderBuilderTest, NoResolutionOrCPR)
{
    YAML::Node config = this->loadTestYaml("/incremental_encoder_no_resolution.yaml");

    ASSERT_THROW(HardwareBuilder::createIncrementalEncoder(config, motor_controller_type), MissingKeyException);
}

TEST_F(IncrementalEncoderBuilderTest, NoTransmission)
{
    YAML::Node config = this->loadTestYaml("/incremental_encoder_no_transmission.yaml");

    ASSERT_THROW(HardwareBuilder::createIncrementalEncoder(config, motor_controller_type), MissingKeyException);
}
#endif
