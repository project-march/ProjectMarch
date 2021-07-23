// Copyright 2019 Project March.
#include "march_hardware_builder/hardware_builder.h"
#include "march_hardware_builder/hardware_config_exceptions.h"

#include <algorithm>
#include <fstream>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <ros/ros.h>

#include <march_hardware/error/error_type.h>
#include <march_hardware/error/hardware_exception.h>
#include <march_hardware/ethercat/pdo_interface.h>
#include <march_hardware/ethercat/sdo_interface.h>
#include <march_hardware/motor_controller/motor_controller_type.h>
#include <march_hardware/motor_controller/odrive/odrive_state.h>
#include <march_hardware/pressure_sole/pressure_sole.h>

const std::vector<std::string> HardwareBuilder::ABSOLUTE_ENCODER_REQUIRED_KEYS
    = { "resolution", "minPositionIU", "maxPositionIU" };
const std::vector<std::string>
    HardwareBuilder::INCREMENTAL_ENCODER_REQUIRED_KEYS
    = { "resolution", "transmission" };
const std::vector<std::string> HardwareBuilder::IMOTIONCUBE_REQUIRED_KEYS
    = { "incrementalEncoder", "absoluteEncoder" };
const std::vector<std::string> HardwareBuilder::ODRIVE_REQUIRED_KEYS
    = { "axis", "incrementalEncoder", "motorKV" };
const std::vector<std::string> HardwareBuilder::TEMPERATUREGES_REQUIRED_KEYS
    = { "slaveIndex", "byteOffset" };
const std::vector<std::string> HardwareBuilder::JOINT_REQUIRED_KEYS
    = { "allowActuation", "motor_controller" };
const std::vector<std::string> HardwareBuilder::MOTOR_CONTROLLER_REQUIRED_KEYS
    = { "slaveIndex", "type" };
const std::vector<std::string> HardwareBuilder::PRESSURE_SOLE_REQUIRED_KEYS
    = { "slaveIndex", "byteOffset", "side" };
const std::vector<std::string>
    HardwareBuilder::POWER_DISTRIBUTION_BOARD_REQUIRED_KEYS
    = { "slaveIndex", "byteOffset" };

HardwareBuilder::HardwareBuilder(AllowedRobot robot,
    bool remove_fixed_joints_from_ethercat_train, std::string if_name)
    : HardwareBuilder(robot.getFilePath(),
        remove_fixed_joints_from_ethercat_train, std::move(if_name))
{
}

HardwareBuilder::HardwareBuilder(AllowedRobot robot, urdf::Model urdf)
    : robot_config_(YAML::LoadFile(robot.getFilePath()))
    , urdf_(std::move(urdf))
    , init_urdf_(false)
{
}

HardwareBuilder::HardwareBuilder(const std::string& yaml_path,
    bool remove_fixed_joints_from_ethercat_train, std::string if_name)
    : robot_config_(YAML::LoadFile(yaml_path))
    , remove_fixed_joints_from_ethercat_train_(
          remove_fixed_joints_from_ethercat_train)
    , if_name_(std::move(if_name))
{
}

HardwareBuilder::HardwareBuilder(const std::string& yaml_path, urdf::Model urdf)
    : robot_config_(YAML::LoadFile(yaml_path))
    , urdf_(std::move(urdf))
    , init_urdf_(false)
{
}

std::unique_ptr<march::MarchRobot> HardwareBuilder::createMarchRobot()
{
    this->initUrdf();
    auto pdo_interface = march::PdoInterfaceImpl::create();
    auto sdo_interface = march::SdoInterfaceImpl::create();
    const auto robot_name
        = this->robot_config_.begin()->first.as<std::string>();

    // Remove top level robot name key
    YAML::Node config = this->robot_config_[robot_name];

    // Read if name from robot config if is an empty value
    if (if_name_ == "") {
        if_name_ = config["if_name"].as<std::string>();
    }

    const auto cycle_time = config["ecatCycleTime"].as<int>();
    const auto slave_timeout = config["ecatSlaveTimeout"].as<int>();

    std::vector<march::Joint> joints
        = this->createJoints(config["joints"], pdo_interface, sdo_interface);

    auto pressure_soles = createPressureSoles(
        config["pressure_soles"], pdo_interface, sdo_interface);
    auto power_distribution_board = createPowerDistributionBoard(
        config["power_distribution_board"], pdo_interface, sdo_interface);
    return std::make_unique<march::MarchRobot>(std::move(joints), this->urdf_,
        std::move(pressure_soles), if_name_, cycle_time, slave_timeout,
        std::make_optional<march::PowerDistributionBoard>(
            power_distribution_board));
}

march::Joint HardwareBuilder::createJoint(const YAML::Node& joint_config,
    const std::string& joint_name, const urdf::JointConstSharedPtr& urdf_joint,
    const march::PdoInterfacePtr& pdo_interface,
    const march::SdoInterfacePtr& sdo_interface)
{
    ROS_DEBUG("Starting creation of joint %s", joint_name.c_str());
    if (!urdf_joint) {
        throw march::error::HardwareException(
            march::error::ErrorType::MISSING_URDF_JOINT,
            "No URDF joint given for joint " + joint_name);
    }
    HardwareBuilder::validateRequiredKeysExist(
        joint_config, HardwareBuilder::JOINT_REQUIRED_KEYS, "joint");

    auto net_number = -1;
    if (joint_config["netNumber"]) {
        net_number = joint_config["netNumber"].as<int>();
    }

    int slaveIndex = joint_config["motor_controller"]["slaveIndex"].as<int>();
    ROS_INFO_STREAM("Joint " << joint_name.c_str()
                             << " will be actuated with slave index "
                             << slaveIndex);

    const auto allow_actuation = joint_config["allowActuation"].as<bool>();

    auto motor_controller = HardwareBuilder::createMotorController(
        joint_config["motor_controller"], urdf_joint, pdo_interface,
        sdo_interface);

    if (joint_config["temperatureges"]) {
        auto ges = HardwareBuilder::createTemperatureGES(
            joint_config["temperatureges"], pdo_interface, sdo_interface);
        return { joint_name, net_number, allow_actuation,
            std::move(motor_controller), std::move(ges) };
    } else {
        return { joint_name, net_number, allow_actuation,
            std::move(motor_controller) };
    }
}

std::unique_ptr<march::MotorController> HardwareBuilder::createMotorController(
    const YAML::Node& config, const urdf::JointConstSharedPtr& urdf_joint,
    const march::PdoInterfacePtr& pdo_interface,
    const march::SdoInterfacePtr& sdo_interface)
{
    HardwareBuilder::validateRequiredKeysExist(config,
        HardwareBuilder::MOTOR_CONTROLLER_REQUIRED_KEYS, "motor_controller");

    march::ActuationMode mode;
    if (config["actuationMode"]) {
        mode = march::ActuationMode(config["actuationMode"].as<std::string>());
    }

    std::unique_ptr<march::MotorController> motor_controller;
    auto type = config["type"].as<std::string>();
    if (type == "imotioncube") {
        motor_controller = createIMotionCube(
            config, mode, urdf_joint, pdo_interface, sdo_interface);
    } else if (type == "odrive") {
        motor_controller = createODrive(
            config, mode, urdf_joint, pdo_interface, sdo_interface);
    } else {
        throw march::error::HardwareException(
            march::error::ErrorType::INVALID_MOTOR_CONTROLLER,
            "Motorcontroller %s not valid", type);
    }
    return motor_controller;
}

std::unique_ptr<march::IMotionCube> HardwareBuilder::createIMotionCube(
    const YAML::Node& imc_config, march::ActuationMode mode,
    const urdf::JointConstSharedPtr& urdf_joint,
    const march::PdoInterfacePtr& pdo_interface,
    const march::SdoInterfacePtr& sdo_interface)
{
    if (!imc_config || !urdf_joint) {
        return nullptr;
    }

    HardwareBuilder::validateRequiredKeysExist(
        imc_config, HardwareBuilder::IMOTIONCUBE_REQUIRED_KEYS, "imotioncube");

    YAML::Node incremental_encoder_config = imc_config["incrementalEncoder"];
    YAML::Node absolute_encoder_config = imc_config["absoluteEncoder"];
    int slave_index = imc_config["slaveIndex"].as<int>();

    std::ifstream imc_setup_data;
    imc_setup_data.open(
        ros::package::getPath("march_hardware")
            .append("/config/sw_files/" + urdf_joint->name + ".sw"));
    std::string setup = convertSWFileToString(imc_setup_data);
    return std::make_unique<march::IMotionCube>(
        march::Slave(slave_index, pdo_interface, sdo_interface),
        HardwareBuilder::createAbsoluteEncoder(absolute_encoder_config,
            march::MotorControllerType::IMotionCube, urdf_joint),
        HardwareBuilder::createIncrementalEncoder(incremental_encoder_config,
            march::MotorControllerType::IMotionCube),
        setup, mode);
}

std::unique_ptr<march::ODrive> HardwareBuilder::createODrive(
    const YAML::Node& odrive_config, march::ActuationMode mode,
    const urdf::JointConstSharedPtr& urdf_joint,
    const march::PdoInterfacePtr& pdo_interface,
    const march::SdoInterfacePtr& sdo_interface)
{
    if (!odrive_config || !urdf_joint) {
        return nullptr;
    }

    HardwareBuilder::validateRequiredKeysExist(
        odrive_config, HardwareBuilder::ODRIVE_REQUIRED_KEYS, "odrive");

    YAML::Node incremental_encoder_config = odrive_config["incrementalEncoder"];
    int slave_index = odrive_config["slaveIndex"].as<int>();

    march::ODriveAxis axis = march::ODriveAxis(odrive_config["axis"].as<int>());

    bool index_found = false;
    if (odrive_config["indexFound"]) {
        index_found = odrive_config["indexFound"].as<bool>();
    }
    auto motor_kv = odrive_config["motorKV"].as<unsigned int>();

    if (odrive_config["absoluteEncoder"]) {
        YAML::Node absolute_encoder_config = odrive_config["absoluteEncoder"];
        return std::make_unique<march::ODrive>(
            march::Slave(slave_index, pdo_interface, sdo_interface), axis,
            HardwareBuilder::createAbsoluteEncoder(absolute_encoder_config,
                march::MotorControllerType::ODrive, urdf_joint),
            HardwareBuilder::createIncrementalEncoder(
                incremental_encoder_config, march::MotorControllerType::ODrive),
            mode, index_found, motor_kv);
    } else {
        return std::make_unique<march::ODrive>(
            march::Slave(slave_index, pdo_interface, sdo_interface), axis,
            /*absolute_encoder=*/nullptr,
            HardwareBuilder::createIncrementalEncoder(
                incremental_encoder_config, march::MotorControllerType::ODrive),
            mode, index_found, motor_kv);
    }
}

std::unique_ptr<march::AbsoluteEncoder> HardwareBuilder::createAbsoluteEncoder(
    const YAML::Node& absolute_encoder_config,
    const march::MotorControllerType motor_controller_type,
    const urdf::JointConstSharedPtr& urdf_joint)
{
    if (!absolute_encoder_config || !urdf_joint) {
        return nullptr;
    }

    HardwareBuilder::validateRequiredKeysExist(absolute_encoder_config,
        HardwareBuilder::ABSOLUTE_ENCODER_REQUIRED_KEYS, "absoluteEncoder");

    const auto resolution = absolute_encoder_config["resolution"].as<size_t>();
    const auto min_position
        = absolute_encoder_config["minPositionIU"].as<int32_t>();
    const auto max_position
        = absolute_encoder_config["maxPositionIU"].as<int32_t>();

    if (!urdf_joint->limits) {
        throw march::error::HardwareException(
            march::error::ErrorType::MISSING_REQUIRED_KEY,
            "Missing limits in the urdf");
    }

    double soft_lower_limit;
    double soft_upper_limit;
    if (urdf_joint->safety) {
        soft_lower_limit = urdf_joint->safety->soft_lower_limit;
        soft_upper_limit = urdf_joint->safety->soft_upper_limit;
    } else {
        ROS_WARN("URDF joint %s has no defined soft limits, so using hard "
                 "limits as soft limits",
            urdf_joint->name.c_str());
        soft_lower_limit = urdf_joint->limits->lower;
        soft_upper_limit = urdf_joint->limits->upper;
    }

    return std::make_unique<march::AbsoluteEncoder>(resolution,
        motor_controller_type, getEncoderDirection(absolute_encoder_config),
        min_position, max_position, urdf_joint->limits->lower,
        urdf_joint->limits->upper, soft_lower_limit, soft_upper_limit);
}

std::unique_ptr<march::IncrementalEncoder>
HardwareBuilder::createIncrementalEncoder(
    const YAML::Node& incremental_encoder_config,
    const march::MotorControllerType motor_controller_type)
{
    if (!incremental_encoder_config) {
        return nullptr;
    }

    HardwareBuilder::validateRequiredKeysExist(incremental_encoder_config,
        HardwareBuilder::INCREMENTAL_ENCODER_REQUIRED_KEYS,
        "incrementalEncoder");

    const auto resolution
        = incremental_encoder_config["resolution"].as<size_t>();
    const auto transmission
        = incremental_encoder_config["transmission"].as<double>();
    return std::make_unique<march::IncrementalEncoder>(resolution,
        motor_controller_type, getEncoderDirection(incremental_encoder_config),
        transmission);
}

march::Encoder::Direction HardwareBuilder::getEncoderDirection(
    const YAML::Node& encoder_config)
{
    if (encoder_config["direction"]) {
        switch (encoder_config["direction"].as<int>()) {
            case 1:
                return march::Encoder::Direction::Positive;
            case -1:
                return march::Encoder::Direction::Negative;
            default:
                throw march::error::HardwareException(
                    march::error::ErrorType::INVALID_ENCODER_DIRECTION);
        }
    } else {
        return march::Encoder::Direction::Positive;
    }
}

std::unique_ptr<march::TemperatureGES> HardwareBuilder::createTemperatureGES(
    const YAML::Node& temperature_ges_config,
    const march::PdoInterfacePtr& pdo_interface,
    const march::SdoInterfacePtr& sdo_interface)
{
    if (!temperature_ges_config) {
        return nullptr;
    }

    HardwareBuilder::validateRequiredKeysExist(temperature_ges_config,
        HardwareBuilder::TEMPERATUREGES_REQUIRED_KEYS, "temperatureges");

    const auto slave_index = temperature_ges_config["slaveIndex"].as<int>();
    const auto byte_offset = temperature_ges_config["byteOffset"].as<int>();
    return std::make_unique<march::TemperatureGES>(
        march::Slave(slave_index, pdo_interface, sdo_interface), byte_offset);
}

void HardwareBuilder::validateRequiredKeysExist(const YAML::Node& config,
    const std::vector<std::string>& key_list, const std::string& object_name)
{
    for (const std::string& key : key_list) {
        if (!config[key]) {
            throw MissingKeyException(key, object_name);
        }
    }
}

void HardwareBuilder::initUrdf()
{
    if (this->init_urdf_) {
        if (!this->urdf_.initParam("/robot_description")) {
            throw march::error::HardwareException(
                march::error::ErrorType::INIT_URDF_FAILED);
        }
        this->init_urdf_ = false;
    }
}

int HardwareBuilder::getSlaveIndexFromJointConfig(const YAML::Node& joint) const
{
    if (joint["motor_controller"]) {
        if (joint["motor_controller"]["slaveIndex"]) {
            return joint["motor_controller"]["slaveIndex"].as<int>();
        }
    }
    return -1;
}

std::set<int> HardwareBuilder::getSlaveIndicesOfFixedJoints(
    const YAML::Node& joints_config) const
{
    std::set<int> fixedSlaveIndices;
    std::set<int> actuatingSlaveIndices;
    for (const YAML::Node& joint_config : joints_config) {
        const auto joint_name = joint_config.begin()->first.as<std::string>();
        const auto urdf_joint = this->urdf_.getJoint(joint_name);
        int slaveIndex = getSlaveIndexFromJointConfig(joint_config[joint_name]);
        if (urdf_joint->type == urdf::Joint::FIXED) {
            ROS_INFO_STREAM("Joint "
                << joint_name << " with slaveIndex " << slaveIndex
                << " is fixed and will be removed from ethercat "
                   "train config");
            if (slaveIndex != -1) {
                fixedSlaveIndices.insert(slaveIndex);
            }
        } else {
            actuatingSlaveIndices.insert(slaveIndex);
        }
    }
    for (int actuatingSlaveIndex : actuatingSlaveIndices) {
        fixedSlaveIndices.erase(actuatingSlaveIndex);
    }
    return fixedSlaveIndices;
}

int HardwareBuilder::updateSlaveIndexBasedOnFixedJoints(
    const YAML::Node& joint_config, const std::string& joint_name,
    const std::set<int>& fixedSlaveIndices) const
{
    int slaveIndex = getSlaveIndexFromJointConfig(joint_config[joint_name]);
    int amountFixedBeforeSlave = 0;
    for (int fixedSlaveIndex : fixedSlaveIndices) {
        if (fixedSlaveIndex < slaveIndex) {
            amountFixedBeforeSlave++;
        }
    }
    return slaveIndex - amountFixedBeforeSlave;
}

std::vector<march::Joint> HardwareBuilder::createJoints(
    const YAML::Node& joints_config,
    const march::PdoInterfacePtr& pdo_interface,
    const march::SdoInterfacePtr& sdo_interface) const
{
    // Use a sorted map to store the joint names and yaml configurations
    std::map<std::string, YAML::Node> actuating_joint_names;

    std::set<int> fixedSlaveIndices;
    if (this->remove_fixed_joints_from_ethercat_train_) {
        fixedSlaveIndices = getSlaveIndicesOfFixedJoints(joints_config);
    }

    for (YAML::Node joint_config : joints_config) {
        const auto joint_name = joint_config.begin()->first.as<std::string>();
        const auto urdf_joint = this->urdf_.getJoint(joint_name);
        if (urdf_joint->type != urdf::Joint::FIXED) {
            if (this->remove_fixed_joints_from_ethercat_train_) {
                joint_config[joint_name]["motor_controller"]["slaveIndex"]
                    = updateSlaveIndexBasedOnFixedJoints(
                        joint_config, joint_name, fixedSlaveIndices);
            }
            actuating_joint_names.insert(
                std::pair<std::string, YAML::Node>(joint_name, joint_config));
        } else {
            ROS_WARN(
                "Joint %s is fixed in the URDF, but defined in the robot yaml",
                joint_name.c_str());
        }
    }

    std::vector<march::Joint> joints;
    std::stringstream ss;
    for (auto& entry : actuating_joint_names) {
        const auto joint_name = entry.first;
        const auto joint_config = entry.second;
        const auto urdf_joint = this->urdf_.getJoint(joint_name);
        joints.push_back(HardwareBuilder::createJoint(joint_config[joint_name],
            joint_name, urdf_joint, pdo_interface, sdo_interface));
        ss << joint_name << ", ";
    }
    ROS_INFO("Sorted actuating joints are: [%s]", ss.str().c_str());

    for (const auto& urdf_joint : this->urdf_.joints_) {
        if (urdf_joint.second->type != urdf::Joint::FIXED) {
            auto equals_joint_name = [&](const auto& joint) {
                return joint.getName() == urdf_joint.first;
            };
            auto result
                = std::find_if(joints.begin(), joints.end(), equals_joint_name);
            if (result == joints.end()) {
                ROS_WARN("Joint %s in URDF is not defined in robot yaml",
                    urdf_joint.first.c_str());
            }
        }
    }

    joints.shrink_to_fit();
    return joints;
}

std::vector<march::PressureSole> HardwareBuilder::createPressureSoles(
    const YAML::Node& pressure_soles_config,
    const march::PdoInterfacePtr& pdo_interface,
    const march::SdoInterfacePtr& sdo_interface)
{
    std::vector<march::PressureSole> pressure_soles;
    if (!pressure_soles_config) {
        return pressure_soles;
    }
    for (const YAML::Node& pressure_sole_config : pressure_soles_config) {
        pressure_soles.push_back(HardwareBuilder::createPressureSole(
            pressure_sole_config[pressure_sole_config.begin()
                                     ->first.as<std::string>()],
            pdo_interface, sdo_interface));
    }
    return pressure_soles;
}

march::PressureSole HardwareBuilder::createPressureSole(
    const YAML::Node& pressure_sole_config,
    const march::PdoInterfacePtr& pdo_interface,
    const march::SdoInterfacePtr& sdo_interface)
{
    HardwareBuilder::validateRequiredKeysExist(pressure_sole_config,
        HardwareBuilder::PRESSURE_SOLE_REQUIRED_KEYS, "pressure_sole");

    const auto slave_index = pressure_sole_config["slaveIndex"].as<int>();
    const auto byte_offset = pressure_sole_config["byteOffset"].as<int>();
    const auto side = pressure_sole_config["side"].as<std::string>();
    return march::PressureSole(
        march::Slave(slave_index, pdo_interface, sdo_interface), byte_offset,
        side);
}

march::PowerDistributionBoard HardwareBuilder::createPowerDistributionBoard(
    const YAML::Node& power_distribution_board_config,
    const march::PdoInterfacePtr& pdo_interface,
    const march::SdoInterfacePtr& sdo_interface)
{
    HardwareBuilder::validateRequiredKeysExist(power_distribution_board_config,
        HardwareBuilder::POWER_DISTRIBUTION_BOARD_REQUIRED_KEYS,
        "power_distribution_board");

    const auto slave_index
        = power_distribution_board_config["slaveIndex"].as<int>();
    const auto byte_offset
        = power_distribution_board_config["byteOffset"].as<int>();

    return march::PowerDistributionBoard(
        march::Slave(slave_index, pdo_interface, sdo_interface), byte_offset);
}

std::string convertSWFileToString(std::ifstream& sw_file)
{
    return std::string(std::istreambuf_iterator<char>(sw_file),
        std::istreambuf_iterator<char>());
}
