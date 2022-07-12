// Copyright 2019 Project March.
#include "march_hardware_builder/hardware_builder.h"
#include "march_hardware_builder/hardware_config_exceptions.h"

#include <string>

#include <gtest/gtest.h>
#include <ros/package.h>
#include <urdf/model.h>

#include <march_hardware/encoder/absolute_encoder.h>
#include <march_hardware/motor_controller/motor_controller_type.h>

class AbsoluteEncoderBuilderTest : public ::testing::Test {
protected:
    std::string base_path;
    urdf::JointSharedPtr joint;
    const march::MotorControllerType motor_controller_type
        = march::MotorControllerType::IMotionCube;

    void SetUp() override
    {
        this->base_path = ros::package::getPath("march_hardware_builder")
                              .append(/*__s=*/"/test/yaml/encoder");
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
    this->joint->limits->upper = 2.0;
    this->joint->safety->soft_lower_limit = 0.1;
    this->joint->safety->soft_upper_limit = 1.9;

    march::AbsoluteEncoder expected = march::AbsoluteEncoder(
        /*resolution=*/16, motor_controller_type, /*lower_limit_iu=*/22134,
        /*upper_limit_iu=*/43436, this->joint->limits->lower,
        this->joint->limits->upper, this->joint->safety->soft_lower_limit,
        this->joint->safety->soft_upper_limit);
    auto created = HardwareBuilder::createAbsoluteEncoder(
        config, motor_controller_type, this->joint);
    ASSERT_EQ(expected, *created);
}

TEST_F(AbsoluteEncoderBuilderTest, NoConfig)
{
    YAML::Node config;
    ASSERT_EQ(nullptr,
        HardwareBuilder::createAbsoluteEncoder(
            config[""], motor_controller_type, this->joint));
}

TEST_F(AbsoluteEncoderBuilderTest, NoResolution)
{
    YAML::Node config
        = this->loadTestYaml("/absolute_encoder_no_resolution.yaml");

    ASSERT_THROW(HardwareBuilder::createAbsoluteEncoder(
                     config, motor_controller_type, this->joint),
        MissingKeyException);
}

TEST_F(AbsoluteEncoderBuilderTest, NoMinPosition)
{
    YAML::Node config
        = this->loadTestYaml("/absolute_encoder_no_min_position.yaml");

    ASSERT_THROW(HardwareBuilder::createAbsoluteEncoder(
                     config, motor_controller_type, this->joint),
        MissingKeyException);
}

TEST_F(AbsoluteEncoderBuilderTest, NoMaxPosition)
{
    YAML::Node config
        = this->loadTestYaml("/absolute_encoder_no_max_position.yaml");

    ASSERT_THROW(HardwareBuilder::createAbsoluteEncoder(
                     config, motor_controller_type, this->joint),
        MissingKeyException);
}
