#ifndef __clang_analyzer__
// Copyright 2023 Project March.
#include "march_hardware_builder/hardware_builder.h"
#include "march_hardware_builder/hardware_config_exceptions.h"

#include <filesystem>
#include <iostream>
#include <string>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <gtest/gtest.h>

#include <march_hardware/motor_controller/motor_controller_type.h>
#include <march_hardware/torque_sensor/torque_sensor.h>

class TorqueSensorBuilderTest : public ::testing::Test {
protected:
    std::string base_path;
    const march::MotorControllerType motor_controller_type = march::MotorControllerType::ODrive;

    void SetUp() override
    {
        this->base_path = ament_index_cpp::get_package_share_directory("march_hardware_builder")
                              .append("/robots/test_yamls/torque_sensor");
    }

    YAML::Node loadTestYaml(const std::string& relative_path)
    {
        return YAML::LoadFile(this->base_path.append(relative_path));
    }
};

TEST_F(TorqueSensorBuilderTest, ValidTorqueSensorEmptyConfig)
{
    YAML::Node config;
    ASSERT_EQ(nullptr, HardwareBuilder::createTorqueSensor(config[""], motor_controller_type));
}

TEST_F(TorqueSensorBuilderTest, ValidTorqueSensor)
{
    YAML::Node config = this->loadTestYaml("/torque_sensor.yaml");
    std::cout << this->base_path << std::endl;
    march::TorqueSensor expected = march::TorqueSensor(motor_controller_type,
        /*max_torque=*/10, /*average_torque*/ 10);
    auto created = HardwareBuilder::createTorqueSensor(config, motor_controller_type);
    ASSERT_EQ(expected, *created);
}
#endif