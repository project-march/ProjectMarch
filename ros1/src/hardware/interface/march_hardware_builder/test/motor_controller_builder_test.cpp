#include "march_hardware_builder/hardware_builder.h"
#include "march_hardware_builder/hardware_config_exceptions.h"

#include <string>

#include <gtest/gtest.h>
#include <ros/package.h>
#include <urdf/model.h>

class MotorControllerBuilderTest : public ::testing::Test {
protected:
    std::string base_path;
    urdf::JointSharedPtr joint;
    march::PdoInterfacePtr pdo_interface;
    march::SdoInterfacePtr sdo_interface;

    void SetUp() override
    {
        this->base_path = ros::package::getPath("march_hardware_builder")
                              .append(/*s=*/"/test/yaml/motor_controller");
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

TEST_F(MotorControllerBuilderTest, NoSlaveIndex)
{
    YAML::Node config = this->loadTestYaml("/no_slave_index.yaml");

    ASSERT_THROW(HardwareBuilder::createMotorController(config, this->joint,
                     this->pdo_interface, this->sdo_interface),
        MissingKeyException);
}

TEST_F(MotorControllerBuilderTest, NoType)
{
    YAML::Node config = this->loadTestYaml("/no_type.yaml");

    ASSERT_THROW(HardwareBuilder::createMotorController(config, this->joint,
                     this->pdo_interface, this->sdo_interface),
        MissingKeyException);
}

TEST_F(MotorControllerBuilderTest, ValidIMotionCube)
{
    YAML::Node config = this->loadTestYaml("/imotioncube_correct.yaml");
    this->joint->limits->lower = 0.0;
    this->joint->limits->upper = 2.0;
    this->joint->safety->soft_lower_limit = 0.1;
    this->joint->safety->soft_upper_limit = 1.9;

    auto created = HardwareBuilder::createMotorController(
        config, this->joint, this->pdo_interface, this->sdo_interface);

    auto absolute_encoder = std::make_unique<march::AbsoluteEncoder>(16, 22134,
        43436, this->joint->limits->lower, this->joint->limits->upper,
        this->joint->safety->soft_lower_limit,
        this->joint->safety->soft_upper_limit);
    auto incremental_encoder
        = std::make_unique<march::IncrementalEncoder>(12, 101.0);
    march::IMotionCube expected(march::Slave(/*slave_index=*/2,
                                    this->pdo_interface, this->sdo_interface),
        std::move(absolute_encoder), std::move(incremental_encoder),
        march::ActuationMode::unknown);
    ASSERT_EQ(expected, *created);
}

TEST_F(MotorControllerBuilderTest, RemoveFixedJointsTest)
{
    std::string yaml_path = this->base_path.append(
        "/test_joint_rotational_remove_fixed_joints.yaml");
    urdf::Model urdf;
    urdf.initFile(ros::package::getPath("march_description")
                      .append(/*__s=*/"/urdf/test_joint_rotational.urdf"));
    HardwareBuilder builder
        = HardwareBuilder(yaml_path, urdf);
    auto robot
        = builder.createMarchRobot();
    ASSERT_EQ(robot->size(), 1);
}