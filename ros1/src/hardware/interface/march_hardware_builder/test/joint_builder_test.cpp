// Copyright 2019 Project March.
#include "march_hardware_builder/hardware_builder.h"
#include "march_hardware_builder/hardware_config_exceptions.h"

#include <string>

#include <gtest/gtest.h>
#include <ros/package.h>
#include <urdf/model.h>

#include <march_hardware/encoder/absolute_encoder.h>
#include <march_hardware/encoder/incremental_encoder.h>
#include <march_hardware/error/hardware_exception.h>
#include <march_hardware/imotioncube/imotioncube.h>

class JointBuilderTest : public ::testing::Test {
protected:
    std::string base_path;
    urdf::JointSharedPtr joint;
    march::PdoInterfacePtr pdo_interface;
    march::SdoInterfacePtr sdo_interface;

    void SetUp() override
    {
        this->base_path = ros::package::getPath("march_hardware_builder")
                              .append("/test/yaml/joint");
        this->joint = std::make_shared<urdf::Joint>();
        this->joint->limits = std::make_shared<urdf::JointLimits>();
        this->joint->safety = std::make_shared<urdf::JointSafety>();
        this->pdo_interface = march::PdoInterfaceImpl::create();
        this->sdo_interface = march::SdoInterfaceImpl::create();
    }

    YAML::Node loadTestYaml(const std::string& relative_path)
    {
        return YAML::LoadFile(this->base_path.append(relative_path));
    }
};

TEST_F(JointBuilderTest, ValidJointHip)
{
    YAML::Node config = this->loadTestYaml("/joint_correct.yaml");
    this->joint->limits->lower = 0.0;
    this->joint->limits->upper = 2.0;
    this->joint->safety->soft_lower_limit = 0.1;
    this->joint->safety->soft_upper_limit = 1.9;

    const std::string name = "test_joint_hip";
    march::Joint created = HardwareBuilder::createJoint(
        config, name, this->joint, this->pdo_interface, this->sdo_interface);

    auto absolute_encoder = std::make_unique<march::AbsoluteEncoder>(16, 22134,
        43436, this->joint->limits->lower, this->joint->limits->upper,
        this->joint->safety->soft_lower_limit,
        this->joint->safety->soft_upper_limit);
    auto incremental_encoder
        = std::make_unique<march::IncrementalEncoder>(12, 50.0);
    auto imc = std::make_unique<march::IMotionCube>(
        march::Slave(2, this->pdo_interface, this->sdo_interface),
        std::move(absolute_encoder), std::move(incremental_encoder),
        march::ActuationMode::unknown);
    auto ges = std::make_unique<march::TemperatureGES>(
        march::Slave(1, this->pdo_interface, this->sdo_interface), 2);
    march::Joint expected(name, -1, true, std::move(imc), std::move(ges));

    ASSERT_EQ(expected, created);
}

TEST_F(JointBuilderTest, ValidNotActuated)
{
    YAML::Node config = this->loadTestYaml("/joint_correct_not_actuated.yaml");
    this->joint->limits->lower = 0.0;
    this->joint->limits->upper = 2.0;
    this->joint->safety->soft_lower_limit = 0.1;
    this->joint->safety->soft_upper_limit = 1.9;

    march::Joint created
        = HardwareBuilder::createJoint(config, "test_joint_hip", this->joint,
            this->pdo_interface, this->sdo_interface);

    auto absolute_encoder = std::make_unique<march::AbsoluteEncoder>(16, 22134,
        43436, this->joint->limits->lower, this->joint->limits->upper,
        this->joint->safety->soft_lower_limit,
        this->joint->safety->soft_upper_limit);
    auto incremental_encoder
        = std::make_unique<march::IncrementalEncoder>(12, 50.0);
    auto imc = std::make_unique<march::IMotionCube>(
        march::Slave(2, this->pdo_interface, this->sdo_interface),
        std::move(absolute_encoder), std::move(incremental_encoder),
        march::ActuationMode::unknown);
    auto ges = std::make_unique<march::TemperatureGES>(
        march::Slave(1, this->pdo_interface, this->sdo_interface), 2);
    march::Joint expected(
        "test_joint_hip", -1, false, std::move(imc), std::move(ges));

    ASSERT_EQ(expected, created);
}

TEST_F(JointBuilderTest, NoActuate)
{
    YAML::Node config = this->loadTestYaml("/joint_no_actuate.yaml");

    ASSERT_THROW(HardwareBuilder::createJoint(config, "test_joint_no_actuate",
                     this->joint, this->pdo_interface, this->sdo_interface),
        MissingKeyException);
}

TEST_F(JointBuilderTest, NoIMotionCube)
{
    YAML::Node config = this->loadTestYaml("/joint_no_imotioncube.yaml");
    march::Joint joint
        = HardwareBuilder::createJoint(config, "test_joint_no_imotioncube",
            this->joint, this->pdo_interface, this->sdo_interface);

    ASSERT_FALSE(joint.hasIMotionCube());
}

TEST_F(JointBuilderTest, NoTemperatureGES)
{
    YAML::Node config = this->loadTestYaml("/joint_no_temperature_ges.yaml");
    this->joint->limits->lower = 0.0;
    this->joint->limits->upper = 0.24;
    this->joint->safety->soft_lower_limit = 0.1;
    this->joint->safety->soft_upper_limit = 0.15;

    ASSERT_NO_THROW(
        HardwareBuilder::createJoint(config, "test_joint_no_temperature_ges",
            this->joint, this->pdo_interface, this->sdo_interface));
}

TEST_F(JointBuilderTest, ValidActuationMode)
{
    YAML::Node config = this->loadTestYaml("/joint_correct_position_mode.yaml");
    this->joint->limits->lower = 0.0;
    this->joint->limits->upper = 2.0;
    this->joint->safety->soft_lower_limit = 0.1;
    this->joint->safety->soft_upper_limit = 1.9;

    march::Joint created
        = HardwareBuilder::createJoint(config, "test_joint_hip", this->joint,
            this->pdo_interface, this->sdo_interface);

    march::Joint expected("test_joint_hip", -1, false,
        std::make_unique<march::IMotionCube>(
            march::Slave(1, this->pdo_interface, this->sdo_interface),
            std::make_unique<march::AbsoluteEncoder>(16, 22134, 43436,
                this->joint->limits->lower, this->joint->limits->upper,
                this->joint->safety->soft_lower_limit,
                this->joint->safety->soft_upper_limit),
            std::make_unique<march::IncrementalEncoder>(12, 50.0),
            march::ActuationMode::position));

    ASSERT_EQ(expected, created);
}

TEST_F(JointBuilderTest, EmptyJoint)
{
    YAML::Node config;
    ASSERT_THROW(HardwareBuilder::createJoint(config, "test_joint_empty",
                     this->joint, this->pdo_interface, this->sdo_interface),
        MissingKeyException);
}

TEST_F(JointBuilderTest, NoUrdfJoint)
{
    YAML::Node config = this->loadTestYaml("/joint_correct.yaml");
    ASSERT_THROW(HardwareBuilder::createJoint(config, "test", nullptr,
                     this->pdo_interface, this->sdo_interface),
        march::error::HardwareException);
}
