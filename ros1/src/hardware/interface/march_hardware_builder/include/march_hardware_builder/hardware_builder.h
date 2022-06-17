// Copyright 2019 Project March.
#ifndef MARCH_HARDWARE_BUILDER_HARDWARE_BUILDER_H
#define MARCH_HARDWARE_BUILDER_HARDWARE_BUILDER_H
#include "march_hardware_builder/allowed_robot.h"

#include <memory>
#include <string>
#include <vector>

#include <urdf/model.h>
#include <yaml-cpp/yaml.h>

#include <march_hardware/encoder/absolute_encoder.h>
#include <march_hardware/encoder/incremental_encoder.h>
#include <march_hardware/ethercat/pdo_interface.h>
#include <march_hardware/ethercat/sdo_interface.h>
#include <march_hardware/joint.h>
#include <march_hardware/march_robot.h>
#include <march_hardware/motor_controller/actuation_mode.h>
#include <march_hardware/motor_controller/imotioncube/imotioncube.h>
#include <march_hardware/motor_controller/motor_controller_type.h>
#include <march_hardware/motor_controller/odrive/odrive.h>
#include <march_hardware/power_distribution_board/power_distribution_board.h>
#include <march_hardware/pressure_sole/pressure_sole.h>
#include <march_hardware/temperature/temperature_ges.h>

/**
 * @brief Creates a MarchRobot from a robot yaml and URDF.
 */
class HardwareBuilder {
public:
    /**
     * @brief Initialises a HardwareBuilder with a robotName enumerator.
     * @details Grabs the .yaml file associated with the robot name.
     */
    explicit HardwareBuilder(AllowedRobot robot,
        bool remove_fixed_joints_from_ethercat_train, std::string if_name);

    /**
     * @brief Initialises with a robot name and URDF.
     */
    HardwareBuilder(AllowedRobot robot, urdf::Model urdf);

    /**
     * @brief Initialises a HardwareBuilder with a path to a .yaml file.
     */
    explicit HardwareBuilder(const std::string& yaml_path,
        bool remove_fixed_joints_from_ethercat_train, std::string if_name);

    /**
     * @brief Initialises with a path to yaml and URDF.
     */
    HardwareBuilder(const std::string& yaml_path, urdf::Model urdf);

    /**
     * @brief Creates a MarchRobot. Loads a URDF from the parameter server
     * on topic `/robot_description` when no URDF was given in the constructor.
     *
     * @throws HardwareConfigException When the urdf could not be loaded from
     * the parameter server
     * @throws MissingKeyException When a required key is missing from the given
     * config
     */
    std::unique_ptr<march::MarchRobot> createMarchRobot();

    /**
     * @brief Loops over all keys in the keyList and check if they exist in the
     * config.
     *
     * @throws MissingKeyException when required keys are missing.
     */
    static void validateRequiredKeysExist(const YAML::Node& config,
        const std::vector<std::string>& key_list,
        const std::string& object_name);

    static march::Joint createJoint(const YAML::Node& joint_config,
        const std::string& joint_name,
        const urdf::JointConstSharedPtr& urdf_joint,
        const march::PdoInterfacePtr& pdo_interface,
        const march::SdoInterfacePtr& sdo_interface);
    static std::unique_ptr<march::AbsoluteEncoder> createAbsoluteEncoder(
        const YAML::Node& absolute_encoder_config,
        const march::MotorControllerType motor_controller_type,
        const urdf::JointConstSharedPtr& urdf_joint);
    static std::unique_ptr<march::IncrementalEncoder> createIncrementalEncoder(
        const YAML::Node& incremental_encoder_config,
        const march::MotorControllerType motor_controller_type);
    static march::Encoder::Direction getEncoderDirection(
        const YAML::Node& encoder_config);
    static std::unique_ptr<march::MotorController> createMotorController(
        const YAML::Node& config, const urdf::JointConstSharedPtr& urdf_joint,
        const march::PdoInterfacePtr& pdo_interface,
        const march::SdoInterfacePtr& sdo_interface);
    static std::unique_ptr<march::IMotionCube> createIMotionCube(
        const YAML::Node& imc_config, march::ActuationMode mode,
        const urdf::JointConstSharedPtr& urdf_joint,
        const march::PdoInterfacePtr& pdo_interface,
        const march::SdoInterfacePtr& sdo_interface);
    static std::unique_ptr<march::ODrive> createODrive(
        const YAML::Node& odrive_config, march::ActuationMode mode,
        const urdf::JointConstSharedPtr& urdf_joint,
        const march::PdoInterfacePtr& pdo_interface,
        const march::SdoInterfacePtr& sdo_interface);
    static std::unique_ptr<march::TemperatureGES> createTemperatureGES(
        const YAML::Node& temperature_ges_config,
        const march::PdoInterfacePtr& pdo_interface,
        const march::SdoInterfacePtr& sdo_interface);

    static std::vector<march::PressureSole> createPressureSoles(
        const YAML::Node& pressure_soles_config,
        const march::PdoInterfacePtr& pdo_interface,
        const march::SdoInterfacePtr& sdo_interface);
    static march::PressureSole createPressureSole(
        const YAML::Node& pressure_sole_config,
        const march::PdoInterfacePtr& pdo_interface,
        const march::SdoInterfacePtr& sdo_interface);
    static std::optional<march::PowerDistributionBoard>
    createPowerDistributionBoard(const YAML::Node& power_distribution_config,
        const march::PdoInterfacePtr& pdo_interface,
        const march::SdoInterfacePtr& sdo_interface);

    /**
     * Initializes the URDF if necessary.
     */
    void initUrdf();

    /**
     * Returns all joints found in the given config.
     * Warns when joints are defined as FIXED in the URDF and when a non-FIXED
     * joint is not contained in the config.
     * @param joints_config YAML node that contains a sequence of joint objects
     * @return list of created joints
     */
    std::vector<march::Joint> createJoints(const YAML::Node& joints_config,
        const march::PdoInterfacePtr& pdo_interface,
        const march::SdoInterfacePtr& sdo_interface) const;

    static const std::vector<std::string>
        INCREMENTAL_ENCODER_REQUIRED_KEYS_WITH_COUNTS_PER_ROTATION;
    static const std::vector<std::string>
        INCREMENTAL_ENCODER_REQUIRED_KEYS_WITH_RESOLUTION;
    static const std::vector<std::string>
        ABSOLUTE_ENCODER_REQUIRED_KEYS_WITH_COUNTS_PER_ROTATION;
    static const std::vector<std::string>
        ABSOLUTE_ENCODER_REQUIRED_KEYS_WITH_RESOLUTION;
    static const std::vector<std::string> IMOTIONCUBE_REQUIRED_KEYS;
    static const std::vector<std::string> ODRIVE_REQUIRED_KEYS;
    static const std::vector<std::string> TEMPERATUREGES_REQUIRED_KEYS;
    static const std::vector<std::string> JOINT_REQUIRED_KEYS;
    static const std::vector<std::string> MOTOR_CONTROLLER_REQUIRED_KEYS;
    static const std::vector<std::string> PRESSURE_SOLE_REQUIRED_KEYS;
    static const std::vector<std::string>
        POWER_DISTRIBUTION_BOARD_REQUIRED_KEYS;

private:
    int updateSlaveIndexBasedOnFixedJoints(const YAML::Node& joint_config,
        const std::string& joint_name,
        const std::set<int>& fixedSlaveIndices) const;
    std::set<int> getSlaveIndicesOfFixedJoints(
        const YAML::Node& joints_config) const;
    int getSlaveIndexFromJointConfig(const YAML::Node& joint_config) const;

    YAML::Node robot_config_;
    urdf::Model urdf_;
    bool init_urdf_ = true;
    bool remove_fixed_joints_from_ethercat_train_ = false;
    std::string if_name_ = "";
};

/**
 * Converts the input filestream object to a stringstream object so that is
 * easier to test for in IMotionCUbe.cpp
 */
std::string convertSWFileToString(std::ifstream& sw_file);

#endif // MARCH_HARDWARE_BUILDER_HARDWARE_BUILDER_H
