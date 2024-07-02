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
    }

    YAML::Node loadTestYaml(const std::string& relative_path)
    {
        return YAML::LoadFile(this->base_path.append(relative_path));
    }
};

TEST_F(AbsoluteEncoderBuilderTest, MissingKeys)
{
    YAML::Node config = this->loadTestYaml("/absolute_encoder_missing_keys.yaml");
    ASSERT_THROW(HardwareBuilder::createAbsoluteEncoder(config, motor_controller_type), MissingKeyException);
}

TEST_F(AbsoluteEncoderBuilderTest, FlippedDirection)
{
    YAML::Node config = this->loadTestYaml("/absolute_encoder_flipped_direction.yaml");
    auto encoder = HardwareBuilder::createAbsoluteEncoder(config, motor_controller_type);

    // The expected values
    double expected_min_position = 24;
    double expected_max_position = 924;
    double expected_zero_position = 624;

    // Call getCorrectLimits with the appropriate arguments
    double actual_min_position = HardwareBuilder::getCorrectLimits(config, "maxPositionIU", 1024, true);
    double actual_max_position = HardwareBuilder::getCorrectLimits(config, "minPositionIU", 1024, true);
    double actual_zero_position = HardwareBuilder::getCorrectLimits(config, "zeroPositionIU", 1024, true);

    ASSERT_EQ(expected_min_position, actual_min_position);
    ASSERT_EQ(expected_max_position, actual_max_position);
    ASSERT_EQ(expected_zero_position, actual_zero_position);
}
#endif
