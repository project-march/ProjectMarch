// Copyright 2019 Project March.
#include "march_hardware_builder/hardware_builder.h"
#include "march_hardware_builder/hardware_config_exceptions.h"


const std::vector<std::string> HardwareBuilder::ABSOLUTE_ENCODER_REQUIRED_KEYS
    = { "minPositionIU", "maxPositionIU", "zeroPositionIU", "lowerSoftLimitMarginRad", "upperSoftLimitMarginRad",
        "lowerErrorSoftLimitMarginRad", "upperErrorSoftLimitMarginRad" };
const std::vector<std::string> HardwareBuilder::INCREMENTAL_ENCODER_REQUIRED_KEYS = { "transmission" };
const std::vector<std::string> HardwareBuilder::TORQUE_SENSOR_REQUIRED_KEYS = { "maxTorque", "averageTorque" };
const std::vector<std::string> HardwareBuilder::ODRIVE_REQUIRED_KEYS = { "axis", "incrementalEncoder", "absoluteEncoder", "torqueSensor" };
const std::vector<std::string> HardwareBuilder::JOINT_REQUIRED_KEYS = { "motor_controller" };
const std::vector<std::string> HardwareBuilder::MOTOR_CONTROLLER_REQUIRED_KEYS = { "slaveIndex"};
const std::vector<std::string> HardwareBuilder::POWER_DISTRIBUTION_BOARD_REQUIRED_KEYS = { "slaveIndex", "byteOffset" };


HardwareBuilder::HardwareBuilder(const std::string& yaml_path)
    : robot_config_(YAML::LoadFile(yaml_path))
    , logger_(std::make_shared<march_logger::RosLogger>("march_hardware_builder"))
    , pdo_interface_(march::PdoInterfaceImpl::create())
    , sdo_interface_(march::SdoInterfaceImpl::create(logger_))
{
    logger_->info(yaml_path.c_str());
    logger_->info("HardwareBuilder created");
}

std::unique_ptr<march::MarchRobot> HardwareBuilder::createMarchRobot(const std::vector<std::string>& active_joint_names)
{
    logger_->info("Creating MarchRobot");
    const auto robot_name = this->robot_config_.begin()->first.as<std::string>();
    const auto& config = this->robot_config_[robot_name];
    network_interface_name_ = config["network_interface_name"].as<std::string>();
    std::vector<march::Joint> joints = this->createJoints(config["joints"], active_joint_names);
    auto power_distribution_board = createPowerDistributionBoard(config["power_distribution_board"]);


    return std::make_unique<march::MarchRobot>(
        /*joint_list_=*/std::move(joints),
        /*logger=*/std::make_shared<march_logger::RosLogger>("march_robot"),
        /*network_interface_name=*/network_interface_name_,
        /*ecatCycleTime=*/config["ecatCycleTime"].as<int>(),
        /*ecatSlaveTimeout=*/config["ecatSlaveTimeout"].as<int>(),
        /*PowerDistributionBoard=*/std::move(power_distribution_board));
}

std::vector<march::Joint> HardwareBuilder::createJoints(const YAML::Node& joints_config, const std::vector<std::string>& active_joint_names) const
{
    std::map<std::string, YAML::Node> joint_configs_map = getMapOfActiveJointConfigs(joints_config, active_joint_names);

    std::vector<march::Joint> joints;
    joints.reserve(active_joint_names.size());

    std::string joint_names_str;
    for (const auto& entry : joint_configs_map) {
        const std::string& joint_name = entry.first;
        const YAML::Node& joint_config = entry.second;
        joints.emplace_back(HardwareBuilder::createJoint(joint_name, joint_config));
        joint_names_str += joint_name + ", ";
    }

    logger_->info(logger_->fstring("Sorted actuating joints are: [%s]", joint_names_str.c_str()));

    joints.shrink_to_fit();
    return joints;
}


march::Joint HardwareBuilder::createJoint(const std::string& joint_name, const YAML::Node& joint_config) const
{
    HardwareBuilder::validateRequiredKeysExist(joint_config, HardwareBuilder::JOINT_REQUIRED_KEYS, "joint " + joint_name);

    auto logger = std::make_shared<march_logger::RosLogger>(joint_name);
    auto motor_controller = HardwareBuilder::createMotorController(*logger, joint_config["motor_controller"]);

    const auto& pos_gains = joint_config["gains"]["position"];
    const auto& tor_gains = joint_config["gains"]["torque"];
    std::array<double, 3> position_gains = {pos_gains["p"].as<double>(), pos_gains["i"].as<double>(), pos_gains["d"].as<double>()};
    std::array<double, 2> torque_gains = {tor_gains["p"].as<double>(), tor_gains["d"].as<double>()};

    return {joint_name, std::move(motor_controller), position_gains, torque_gains, logger};
}


std::map<std::string, YAML::Node> HardwareBuilder::getMapOfActiveJointConfigs(const YAML::Node& joints_config, std::vector<std::string> active_joint_names) const
{
    // Use a sorted map to store the joint names and yaml configurations
    std::map<std::string, YAML::Node> joint_configs_map;
    
    for (const auto& joint_config : joints_config) {
        const auto joint_name = joint_config.begin()->first.as<std::string>();
        auto found_position = std::find(active_joint_names.begin(), active_joint_names.end(), joint_name);
        if (found_position != active_joint_names.end()) {
            joint_configs_map.emplace(joint_name, joint_config[joint_name]);
            active_joint_names.erase(found_position);
        } else {
            logger_->warn(logger_->fstring("Joint %s is not in the controller yaml, but is defined in the robot yaml.", joint_name.c_str()));
        }
    }

    // Warn if not all active joints have a config.
    if (!active_joint_names.empty()) {
        std::string active_joint_names_str;
        for (const auto& active_joint_without_config : active_joint_names) {
            active_joint_names_str += active_joint_without_config + ", ";
        }
        logger_->warn(logger_->fstring("Joints [%s] are defined in the controller yaml, but not in the robot yaml.",active_joint_names_str.c_str()));
    }

    return joint_configs_map;
}

// Actuation mode is currently not used, but may be used in the future
std::unique_ptr<march::MotorController> HardwareBuilder::createMotorController(const march_logger::BaseLogger& logger, const YAML::Node& config) const
{
    HardwareBuilder::validateRequiredKeysExist(config, HardwareBuilder::MOTOR_CONTROLLER_REQUIRED_KEYS, "motor_controller");

    march::ActuationMode mode;
    if (config["actuationMode"]) {
        mode = march::ActuationMode(config["actuationMode"].as<std::string>(), *logger.get_logger_append_suffix("actuation_mode"));
    }

    std::unique_ptr<march::MotorController> motor_controller = createODrive(logger, config, mode);

    return motor_controller;
}

// TODO: look into reintegration of the index found
std::unique_ptr<march::ODrive> HardwareBuilder::createODrive(const march_logger::BaseLogger& logger, const YAML::Node& odrive_config, march::ActuationMode mode) const
{
    HardwareBuilder::validateRequiredKeysExist(odrive_config, HardwareBuilder::ODRIVE_REQUIRED_KEYS, "odrive");

    const auto slave_index = odrive_config["slaveIndex"].as<int>();
    march::ODriveAxis axis = march::ODriveAxis(odrive_config["axis"].as<int>());
    const auto& absolute_encoder_config = odrive_config["absoluteEncoder"];
    const auto& use_inc_enc_for_position_config = odrive_config["useIncrementalEncoderForPosition"];
    const auto& incremental_encoder_config = odrive_config["incrementalEncoder"];
    const auto& torque_sensor_config = odrive_config["torqueSensor"];
  
    auto loggerPtr = std::shared_ptr<march_logger::BaseLogger>(logger.get_logger_append_suffix(".Odrive"));
    bool index_found = false;
    
    return std::make_unique<march::ODrive>(
        /*slave=*/march::Slave(slave_index, pdo_interface_, sdo_interface_),
        /*axis=*/axis,
        /*absolute_encoder=*/HardwareBuilder::createAbsoluteEncoder(absolute_encoder_config, march::MotorControllerType::ODrive),
        /*incremental_encoder=*/HardwareBuilder::createIncrementalEncoder(incremental_encoder_config, march::MotorControllerType::ODrive),
        /*torque_sensor=*/HardwareBuilder::createTorqueSensor(torque_sensor_config, march::MotorControllerType::ODrive),
        /*actuation_mode=*/mode,
        /*use_inc_enc_for_position=*/use_inc_enc_for_position_config,
        /*index_found=*/index_found,
        /*logger=*/loggerPtr);
}

std::unique_ptr<march::AbsoluteEncoder> HardwareBuilder::createAbsoluteEncoder(const YAML::Node& absolute_encoder_config, const march::MotorControllerType motor_controller_type)
{
    HardwareBuilder::validateRequiredKeysExist(absolute_encoder_config, HardwareBuilder::ABSOLUTE_ENCODER_REQUIRED_KEYS, "absoluteEncoder");

    const auto counts_per_rotation = validate_and_get_counts_per_rotation(absolute_encoder_config);
    const auto min_position = absolute_encoder_config["minPositionIU"].as<int32_t>();
    const auto max_position = absolute_encoder_config["maxPositionIU"].as<int32_t>();
    const auto zero_position = absolute_encoder_config["zeroPositionIU"].as<int32_t>();
    const auto lower_soft_limit_margin = absolute_encoder_config["lowerSoftLimitMarginRad"].as<double>();
    const auto upper_soft_limit_margin = absolute_encoder_config["upperSoftLimitMarginRad"].as<double>();
    const auto lower_error_soft_limit_margin = absolute_encoder_config["lowerErrorSoftLimitMarginRad"].as<double>();
    const auto upper_error_soft_limit_margin = absolute_encoder_config["upperErrorSoftLimitMarginRad"].as<double>();

    return std::make_unique<march::AbsoluteEncoder>(
        /*counts_per_rotation=*/counts_per_rotation,
        /*motor_controller_type=*/motor_controller_type,
        /*direction=*/getEncoderDirection(absolute_encoder_config),
        /*lower_limit_iu=*/min_position,
        /*upper_limit_iu=*/max_position,
        /*zero_position_iu=*/zero_position,
        /*lower_error_soft_limit_rad_diff=*/lower_error_soft_limit_margin,
        /*upper_error_soft_limit_rad_diff=*/upper_error_soft_limit_margin,
        /*lower_soft_limit_rad_diff=*/lower_soft_limit_margin,
        /*upper_soft_limit_rad_diff=*/upper_soft_limit_margin);
}

std::unique_ptr<march::IncrementalEncoder> HardwareBuilder::createIncrementalEncoder(const YAML::Node& incremental_encoder_config, const march::MotorControllerType motor_controller_type)
{
    HardwareBuilder::validateRequiredKeysExist( incremental_encoder_config, HardwareBuilder::INCREMENTAL_ENCODER_REQUIRED_KEYS, "incrementalEncoder");

    const auto counts_per_rotation = validate_and_get_counts_per_rotation(incremental_encoder_config);
    const auto transmission = incremental_encoder_config["transmission"].as<double>();

    return std::make_unique<march::IncrementalEncoder>(
        /*counts_per_rotation=*/counts_per_rotation,
        /*motor_controller_type=*/motor_controller_type,
        /*direction=*/getEncoderDirection(incremental_encoder_config),
        /*transmission=*/transmission);
}

march::Encoder::Direction HardwareBuilder::getEncoderDirection(const YAML::Node& encoder_config)
{
    if (encoder_config["direction"]) {
        switch (encoder_config["direction"].as<int>()) {
            case 1:
                return march::Encoder::Direction::Positive;
            case -1:
                return march::Encoder::Direction::Negative;
            default:
                throw march::error::HardwareException(march::error::ErrorType::INVALID_ENCODER_DIRECTION);
        }
    } else {
        return march::Encoder::Direction::Positive;
    }
}


std::unique_ptr<march::TorqueSensor> HardwareBuilder::createTorqueSensor(const YAML::Node& torque_sensor_config, const march::MotorControllerType motor_controller_type)
{
    HardwareBuilder::validateRequiredKeysExist(torque_sensor_config, HardwareBuilder::TORQUE_SENSOR_REQUIRED_KEYS, "torqueSensor");

    const auto max_torque = torque_sensor_config["maxTorque"].as<float>();
    const auto average_torque = torque_sensor_config["averageTorque"].as<float>();

    return std::make_unique<march::TorqueSensor>(
        /*motor_controller_type*/ motor_controller_type,
        /*max_torque*/ max_torque,
        /*average_torque*/ average_torque);
}

std::optional<march::PowerDistributionBoard> HardwareBuilder::createPowerDistributionBoard(const YAML::Node& power_distribution_board_config) const
{
    if (!power_distribution_board_config) {
        return std::nullopt;
    }
    
    HardwareBuilder::validateRequiredKeysExist(power_distribution_board_config,HardwareBuilder::POWER_DISTRIBUTION_BOARD_REQUIRED_KEYS, "power_distribution_board");

    const auto slave_index = power_distribution_board_config["slaveIndex"].as<int>();
    const auto byte_offset = power_distribution_board_config["byteOffset"].as<int>();
   
    return std::make_optional<march::PowerDistributionBoard>(march::Slave(slave_index, pdo_interface_, sdo_interface_), byte_offset);
}

void HardwareBuilder::validateRequiredKeysExist(const YAML::Node& config, const std::vector<std::string>& key_list, const std::string& object_name)
{
    for (const std::string& key : key_list) {
        if (!config[key]) {
            throw MissingKeyException(key, object_name);
        }
    }
}

size_t HardwareBuilder::validate_and_get_counts_per_rotation(const YAML::Node& config)
{
    if (config["countsPerRotation"]) {
        return config["countsPerRotation"].as<size_t>();
    } else if (config["resolution"]) {
        return (size_t)1 << config["resolution"].as<size_t>();
    } else {
        throw MissingKeyException("resolution' or 'countsPerRotation", "incrementalEncoder");
    }
}