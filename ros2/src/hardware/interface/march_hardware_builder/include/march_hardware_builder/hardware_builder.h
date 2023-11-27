// Copyright 2019 Project March.
#ifndef MARCH_HARDWARE_BUILDER_HARDWARE_BUILDER_H
#define MARCH_HARDWARE_BUILDER_HARDWARE_BUILDER_H

#include <algorithm>
#include <memory>
#include <string>
#include <vector>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include <yaml-cpp/yaml.h>

#include <march_hardware/encoder/absolute_encoder.h>
#include <march_hardware/encoder/incremental_encoder.h>
#include <march_hardware/ethercat/pdo_interface.h>
#include <march_hardware/ethercat/sdo_interface.h>
#include <march_hardware/joint.h>
#include <march_hardware/march_robot.h>
#include <march_hardware/motor_controller/actuation_mode.h>
#include <march_hardware/motor_controller/motor_controller_type.h>
#include <march_hardware/motor_controller/odrive/odrive.h>
#include <march_hardware/power_distribution_board/power_distribution_board.h>
#include <march_hardware/temperature/temperature_ges.h>
#include <march_hardware/torque_sensor/torque_sensor.h>
#include <march_logger_cpp/base_logger.hpp>
#include <march_hardware/error/error_type.h>
#include <march_hardware/error/hardware_exception.h>
#include <march_hardware/ethercat/pdo_interface.h>
#include <march_hardware/ethercat/sdo_interface.h>
#include <march_hardware/motor_controller/motor_controller_type.h>
#include <march_hardware/motor_controller/odrive/odrive_state.h>
#include <march_logger_cpp/ros_logger.hpp>

/**
 * @brief Creates a MarchRobot from a robot yaml and URDF.
 */
class HardwareBuilder {
public:
    /**
     * @brief Initialises with a robot name and URDF.
     */
    explicit HardwareBuilder(const std::string& yaml_path);

    /**
     * @brief Creates a march::MarchRobot.
     *
     * @throws HardwareConfigException When the urdf could not be loaded from
     * the parameter server
     * @throws MissingKeyException When a required key is missing from the given
     * config
     */
    std::unique_ptr<march::MarchRobot> createMarchRobot(const std::vector<std::string>& active_joint_names);

    /**
     * Returns all joints found in the given config.
     * Warns when joints are defined as FIXED in the URDF and when a non-FIXED
     * joint is not contained in the config.
     * @param joints_config YAML node that contains a sequence of joint objects
     * @return list of created joints
     */
    std::vector<march::Joint> createJoints(
        const YAML::Node& joints_config, const std::vector<std::string>& active_joint_names) const;

    std::map<std::string, YAML::Node> getMapOfActiveJointConfigs(
        const YAML::Node& joints_config, std::vector<std::string> active_joint_names) const;

    march::Joint createJoint(const std::string& joint_name, const YAML::Node& joint_config) const;
    std::unique_ptr<march::MotorController> createMotorController(
        const march_logger::BaseLogger& parent_joint_name, const YAML::Node& config) const;
    std::unique_ptr<march::ODrive> createODrive(
        const march_logger::BaseLogger& logger, const YAML::Node& odrive_config, march::ActuationMode mode) const;

    static std::unique_ptr<march::AbsoluteEncoder> createAbsoluteEncoder(
        const YAML::Node& absolute_encoder_config, const march::MotorControllerType motor_controller_type);
    static std::unique_ptr<march::IncrementalEncoder> createIncrementalEncoder(
        const YAML::Node& incremental_encoder_config, const march::MotorControllerType motor_controller_type);
    static march::Encoder::Direction getEncoderDirection(const YAML::Node& encoder_config);

    static std::unique_ptr<march::TorqueSensor> createTorqueSensor(
        const YAML::Node& torque_sensor_config, const march::MotorControllerType motor_controller_type);

    std::unique_ptr<march::TemperatureGES> createTemperatureGES(const YAML::Node& temperature_ges_config) const;
    std::optional<march::PowerDistributionBoard> createPowerDistributionBoard(
        const YAML::Node& power_distribution_config) const;

    /**
     * @brief Loops over all keys in the keyList and check if they exist in the
     * config.
     *
     * @throws MissingKeyException when required keys are missing.
     */
    static void validateRequiredKeysExist(
        const YAML::Node& config, const std::vector<std::string>& key_list, const std::string& object_name);

    /**
     * @brief Loops over all keys in the keyList and check if they exist in the
     * config.
     *
     * @throws MissingKeyException when required keys are missing.
     */
    static size_t validate_and_get_counts_per_rotation(const YAML::Node& config);

    static const std::vector<std::string> INCREMENTAL_ENCODER_REQUIRED_KEYS;
    static const std::vector<std::string> ABSOLUTE_ENCODER_REQUIRED_KEYS;
    static const std::vector<std::string> TORQUE_SENSOR_REQUIRED_KEYS;
    static const std::vector<std::string> ODRIVE_REQUIRED_KEYS;
    static const std::vector<std::string> TEMPERATUREGES_REQUIRED_KEYS;
    static const std::vector<std::string> JOINT_REQUIRED_KEYS;
    static const std::vector<std::string> MOTOR_CONTROLLER_REQUIRED_KEYS;
    static const std::vector<std::string> POWER_DISTRIBUTION_BOARD_REQUIRED_KEYS;

private:
    YAML::Node robot_config_;
    std::string if_name_;
    std::shared_ptr<march_logger::BaseLogger> logger_;
    const march::PdoInterfacePtr pdo_interface_;
    const march::SdoInterfacePtr sdo_interface_;
};

/**
 * Converts the input filestream object to a stringstream object so that is
 * easier to test for in IMotionCUbe.cpp
 */
std::string convertSWFileToString(std::ifstream& sw_file);

#endif // MARCH_HARDWARE_BUILDER_HARDWARE_BUILDER_H
