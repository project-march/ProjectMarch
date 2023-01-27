#ifndef __clang_analyzer__
// Copyright 2019 Project March.
#include "march_hardware_builder/hardware_builder.h"
#include "march_hardware_builder/hardware_config_exceptions.h"

#include <string>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <gtest/gtest.h>
#include <urdf/model.h>

#include <march_hardware/encoder/absolute_encoder.h>
#include <march_hardware/motor_controller/motor_controller_type.h>

class AbsoluteEncoderBuilderTest : public ::testing::Test {
protected:
    std::string base_path;
    urdf::JointSharedPtr joint;
    const march::MotorControllerType motor_controller_type = march::MotorControllerType::ODrive;

    void SetUp() override
    {
        this->base_path = ament_index_cpp::get_package_share_directory("march_hardware_builder")
                              .append("/robots/test_yamls/encoder");
        this->joint = std::make_shared<urdf::Joint>();
        this->joint->limits = std::make_shared<urdf::JointLimits>();
        this->joint->safety = std::make_shared<urdf::JointSafety>();
    }

    YAML::Node loadTestYaml(const std::string& relative_path)
    {
        return YAML::LoadFile(this->base_path.append(relative_path));
    }
};

TEST_F(AbsoluteEncoderBuilderTest, ValidEncoderHip)
{
    YAML::Node config = this->loadTestYaml("/absolute_encoder_correct.yaml");
    this->joint->limits->lower = 0.0;
    this->joint->limits->upper = 1.9;
    this->joint->safety->soft_lower_limit = 0.01;
    this->joint->safety->soft_upper_limit = 2.0;

    march::AbsoluteEncoder expected = march::AbsoluteEncoder(
        /*counts_per_rotation=*/(size_t)1 << 16, motor_controller_type,
        /*lower_limit_iu=*/22134,
        /*upper_limit_iu=*/43436,
        /*zero_position_iu*/ 30000, this->joint->limits->lower, this->joint->limits->upper,
        this->joint->safety->soft_lower_limit, this->joint->safety->soft_upper_limit);
    auto created = HardwareBuilder::createAbsoluteEncoder(config, motor_controller_type);
    ASSERT_EQ(expected, *created);
}

TEST_F(AbsoluteEncoderBuilderTest, NoConfig)
{
    YAML::Node config;
    ASSERT_EQ(nullptr, HardwareBuilder::createAbsoluteEncoder(config[""], motor_controller_type));
}

TEST_F(AbsoluteEncoderBuilderTest, NoResolutionOrCPR)
{
    YAML::Node config = this->loadTestYaml("/absolute_encoder_no_resolution.yaml");

    ASSERT_THROW(HardwareBuilder::createAbsoluteEncoder(config, motor_controller_type), MissingKeyException);
}

TEST_F(AbsoluteEncoderBuilderTest, NoMinPosition)
{
    YAML::Node config = this->loadTestYaml("/absolute_encoder_no_min_position.yaml");

    ASSERT_THROW(HardwareBuilder::createAbsoluteEncoder(config, motor_controller_type), MissingKeyException);
}

TEST_F(AbsoluteEncoderBuilderTest, NoMaxPosition)
{
    YAML::Node config = this->loadTestYaml("/absolute_encoder_no_max_position.yaml");

    ASSERT_THROW(HardwareBuilder::createAbsoluteEncoder(config, motor_controller_type), MissingKeyException);
}
#endif
