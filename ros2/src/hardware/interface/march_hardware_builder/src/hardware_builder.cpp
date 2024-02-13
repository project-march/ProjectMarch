// Copyright 2019 Project March.
#include "march_hardware_builder/hardware_builder.h"
#include "march_hardware_builder/hardware_config_exceptions.h"



const std::vector<std::string> HardwareBuilder::ABSOLUTE_ENCODER_REQUIRED_KEYS
    = { "minPositionIU", "maxPositionIU", "zeroPositionIU", "lowerSoftLimitMarginRad", "upperSoftLimitMarginRad",
          "lowerErrorSoftLimitMarginRad", "upperErrorSoftLimitMarginRad" };
const std::vector<std::string> HardwareBuilder::INCREMENTAL_ENCODER_REQUIRED_KEYS = { "transmission" };
const std::vector<std::string> HardwareBuilder::TORQUE_SENSOR_REQUIRED_KEYS = { "maxTorque", "averageTorque" };
const std::vector<std::string> HardwareBuilder::ODRIVE_REQUIRED_KEYS = { "axis", "incrementalEncoder", "motorKV" };
const std::vector<std::string> HardwareBuilder::TEMPERATUREGES_REQUIRED_KEYS = { "slaveIndex", "byteOffset" };
const std::vector<std::string> HardwareBuilder::JOINT_REQUIRED_KEYS = { "motor_controller" };
const std::vector<std::string> HardwareBuilder::MOTOR_CONTROLLER_REQUIRED_KEYS = { "slaveIndex", "type" };
const std::vector<std::string> HardwareBuilder::POWER_DISTRIBUTION_BOARD_REQUIRED_KEYS = { "slaveIndex", "byteOffset" };

HardwareBuilder::HardwareBuilder(const std::string& yaml_path)
    : robot_config_(YAML::LoadFile(yaml_path))
    , logger_(std::make_shared<march_logger::RosLogger>("march_hardware_builder"))
    , pdo_interface_(march::PdoInterfaceImpl::create())
    , sdo_interface_(march::SdoInterfaceImpl::create(logger_))
{
    logger_->info(yaml_path.c_str());
}

std::unique_ptr<march::MarchRobot> HardwareBuilder::createMarchRobot(const std::vector<std::string>& active_joint_names)
{
    logger_->info("Starting...");
    const auto robot_name = this->robot_config_.begin()->first.as<std::string>();

    // Remove top level robot name key
    logger_->info(logger_->fstring("Got robot name: %s.", robot_name.c_str()));
    YAML::Node config = this->robot_config_[robot_name];

    if_name_ = config["if_name"].as<std::string>();

    std::vector<march::Joint> joints = this->createJoints(config["joints"], active_joint_names);
    logger_->info("Finished creating joints.");
    auto power_distribution_board = createPowerDistributionBoard(config["power_distribution_board"]);
    logger_->info("Finished.");
    return std::make_unique<march::MarchRobot>(
        /*joint_list_=*/std::move(joints),
        /*logger=*/std::make_shared<march_logger::RosLogger>("march_robot"),
        /*if_name=*/if_name_,
        /*ecatCycleTime=*/config["ecatCycleTime"].as<int>(),
        /*ecatSlaveTimeout=*/config["ecatSlaveTimeout"].as<int>(),
        /*PowerDistributionBoard=*/std::move(power_distribution_board));
}

std::vector<march::Joint> HardwareBuilder::createJoints(
    const YAML::Node& joints_config, const std::vector<std::string>& active_joint_names) const
{

    std::map<std::string, YAML::Node> joint_configs_map = getMapOfActiveJointConfigs(joints_config, active_joint_names);

    std::vector<march::Joint> joints;
    joints.reserve(active_joint_names.size());
    std::stringstream ss;
    for (auto& entry : joint_configs_map) {
        const std::string joint_name = entry.first;
        if (std::find(active_joint_names.begin(), active_joint_names.end(), joint_name) != active_joint_names.end()) {
            const YAML::Node joint_config = entry.second;
            joints.push_back(HardwareBuilder::createJoint(joint_name, joint_config));
            ss << joint_name << ", ";

            RCLCPP_INFO(rclcpp::get_logger("hardware_builder"), "done creating joint.");
        }
    }
    logger_->info(logger_->fstring("Sorted actuating joints are: [%s]", ss.str().c_str()));

    joints.shrink_to_fit();
    return joints;
}

march::Joint HardwareBuilder::createJoint(const std::string& joint_name, const YAML::Node& joint_config) const
{
    logger_->debug(logger_->fstring("Starting creation of joint %s", joint_name.c_str()));
    HardwareBuilder::validateRequiredKeysExist(
        joint_config, HardwareBuilder::JOINT_REQUIRED_KEYS, "joint " + joint_name);

    auto net_number = -1;
    if (joint_config["netNumber"]) {
        net_number = joint_config["netNumber"].as<int>();
    }

    int slaveIndex = joint_config["motor_controller"]["slaveIndex"].as<int>();
    logger_->info(logger_->fstring("Joint %s will be actuated with slave index %i", joint_name.c_str(), slaveIndex));
    auto logger = std::make_shared<march_logger::RosLogger>(joint_name);

    auto motor_controller = HardwareBuilder::createMotorController(*logger, joint_config["motor_controller"]);

        RCLCPP_INFO(rclcpp::get_logger("hardware_builder"), "done creating Motor controllers.");
        std::array<double, 3> position_pid;
        auto pos_pids = joint_config["pids"]["position"];
        position_pid[0] = pos_pids["p"].as<double>();
        position_pid[1] = pos_pids["i"].as<double>();
        position_pid[2] = pos_pids["d"].as<double>();

        std::array<double, 3> torque_pid;
        auto tor_pids = joint_config["pids"]["torque"];
        torque_pid[0] = tor_pids["p"].as<double>();
        torque_pid[1] = tor_pids["i"].as<double>();
        torque_pid[2] = tor_pids["d"].as<double>();

        RCLCPP_INFO(rclcpp::get_logger("hardware_builder"), "done setting pids.");
        if (joint_config["temperatureges"]) {
            auto ges = HardwareBuilder::createTemperatureGES(joint_config["temperatureges"]);
            return { joint_name, net_number, std::move(motor_controller), position_pid, torque_pid, std::move(ges), logger };
        } else {
            return { joint_name, net_number, std::move(motor_controller), position_pid, torque_pid, logger };
        }
    }


std::map<std::string, YAML::Node> HardwareBuilder::getMapOfActiveJointConfigs(
    const YAML::Node& joints_config, std::vector<std::string> active_joint_names) const
{
    // Use a sorted map to store the joint names and yaml configurations
    std::map<std::string, YAML::Node> joint_configs_map;
    for (YAML::Node joint_config : joints_config) {
        const auto joint_name = joint_config.begin()->first.as<std::string>();
        auto found_position = std::find(active_joint_names.begin(), active_joint_names.end(), joint_name);
        if (found_position != active_joint_names.end()) {
            joint_configs_map.insert(std::pair<std::string, YAML::Node>(joint_name, joint_config[joint_name]));
            active_joint_names.erase(found_position);
        } else {
            logger_->warn(logger_->fstring(
                "Joint %s is not in the controller yaml, but is defined in the robot yaml.", joint_name.c_str()));
        }
    }

    // Warn if not all active joints have a config.
    if (!active_joint_names.empty()) {
        std::stringstream active_joint_names_ss;
        for (const auto& active_joint_without_config : active_joint_names) {
            active_joint_names_ss << active_joint_without_config << ", ";
        }
        logger_->warn(logger_->fstring("Joints [%s] are defined in the controller yaml, but not in the robot yaml.",
            active_joint_names_ss.str().c_str()));
    }

    return joint_configs_map;
}

std::unique_ptr<march::MotorController> HardwareBuilder::createMotorController(
    const march_logger::BaseLogger& logger, const YAML::Node& config) const
{
    HardwareBuilder::validateRequiredKeysExist(
        config, HardwareBuilder::MOTOR_CONTROLLER_REQUIRED_KEYS, "motor_controller");

    march::ActuationMode mode;
    if (config["actuationMode"]) {
        mode = march::ActuationMode(
            config["actuationMode"].as<std::string>(), *logger.get_logger_append_suffix("actuation_mode"));
    }

    std::unique_ptr<march::MotorController> motor_controller;
    auto type = config["type"].as<std::string>();
    if (type == "imotioncube") {
        throw march::error::HardwareException(
            march::error::ErrorType::INVALID_MOTOR_CONTROLLER, "Imotioncube is not supported anymore.");
    } else if (type == "odrive") {
        logger_->info("Creating Odrive.");
        motor_controller = createODrive(logger, config, mode);

        RCLCPP_INFO(rclcpp::get_logger("hardware_builder"), "done creating ODrives.");
    } else {
        throw march::error::HardwareException(
            march::error::ErrorType::INVALID_MOTOR_CONTROLLER, "Motorcontroller %s not valid", type);
    }
    return motor_controller;
}

std::unique_ptr<march::ODrive> HardwareBuilder::createODrive(
    const march_logger::BaseLogger& logger, const YAML::Node& odrive_config, march::ActuationMode mode) const
{
    if (!odrive_config) {
        return nullptr;
    }

    logger_->info("ODrive not null");

    HardwareBuilder::validateRequiredKeysExist(odrive_config, HardwareBuilder::ODRIVE_REQUIRED_KEYS, "odrive");
    logger_->info("ODrive has all required keys");

    YAML::Node incremental_encoder_config = odrive_config["incrementalEncoder"];
    YAML::Node torque_sensor_config = odrive_config["torqueSensor"];
    int slave_index = odrive_config["slaveIndex"].as<int>();

    march::ODriveAxis axis = march::ODriveAxis(odrive_config["axis"].as<int>());

    bool index_found = false;
    if (odrive_config["indexFound"]) {
        index_found = odrive_config["indexFound"].as<bool>();
    }
    auto use_incremental_encoder = odrive_config["useIncrementalEncoder"].as<bool>();
    auto motor_kv = odrive_config["motorKV"].as<unsigned int>();
    //    auto incremental_encoder =
    //    std::unique_ptr<march::AbsoluteEncoder>* absolute_encoder = nullptr;
    auto loggerPtr = std::shared_ptr<march_logger::BaseLogger>(logger.get_logger_append_suffix(".Odrive"));
    logger_->info("Starting to create and return encoders.");
    if (odrive_config["absoluteEncoder"]) {
        YAML::Node absolute_encoder_config = odrive_config["absoluteEncoder"];
        return std::make_unique<march::ODrive>(
            /*slave=*/march::Slave(slave_index, pdo_interface_, sdo_interface_),
            /*axis=*/axis,
            /*absolute_encoder=*/
            HardwareBuilder::createAbsoluteEncoder(absolute_encoder_config, march::MotorControllerType::ODrive),
            /*incremental_encoder=*/
            HardwareBuilder::createIncrementalEncoder(incremental_encoder_config, march::MotorControllerType::ODrive),
            /*torque_sensor=*/
            HardwareBuilder::createTorqueSensor(torque_sensor_config, march::MotorControllerType::ODrive),
            /*actuation_mode=*/mode,
            /*index_found=*/index_found,
            /*motor_kv=*/motor_kv,
            /*is_incremental_encoder_more_precise=*/use_incremental_encoder,
            /*logger=*/loggerPtr);
    } else {
        return std::make_unique<march::ODrive>(
            /*slave=*/march::Slave(slave_index, pdo_interface_, sdo_interface_),
            /*axis=*/axis,
            /*absolute_encoder=*/nullptr,
            /*incremental_encoder=*/
            HardwareBuilder::createIncrementalEncoder(incremental_encoder_config, march::MotorControllerType::ODrive),
            /*torque_sensor=*/
            HardwareBuilder::createTorqueSensor(torque_sensor_config, march::MotorControllerType::ODrive),
            /*actuation_mode=*/mode,
            /*index_found=*/index_found,
            /*motor_kv=*/motor_kv,
            /*is_incremental_encoder_more_precise=*/use_incremental_encoder,
            /*logger=*/loggerPtr);
    }
}

std::unique_ptr<march::AbsoluteEncoder> HardwareBuilder::createAbsoluteEncoder(
    const YAML::Node& absolute_encoder_config, const march::MotorControllerType motor_controller_type)
{
    RCLCPP_INFO(rclcpp::get_logger("hardware_builder"), "Creating absoluteEncoder.");
    if (!absolute_encoder_config) {
        return nullptr;
    }

    HardwareBuilder::validateRequiredKeysExist(
        absolute_encoder_config, HardwareBuilder::ABSOLUTE_ENCODER_REQUIRED_KEYS, "absoluteEncoder");

    RCLCPP_INFO(rclcpp::get_logger("hardware_builder"), "absoluteEncoder has required keys.");

    const auto counts_per_rotation = validate_and_get_counts_per_rotation(absolute_encoder_config);
    const auto min_position = absolute_encoder_config["minPositionIU"].as<int32_t>();
    const auto max_position = absolute_encoder_config["maxPositionIU"].as<int32_t>();
    const auto zero_position = absolute_encoder_config["zeroPositionIU"].as<int32_t>();
    const auto lower_soft_limit_margin = absolute_encoder_config["lowerSoftLimitMarginRad"].as<double>();
    const auto upper_soft_limit_margin = absolute_encoder_config["upperSoftLimitMarginRad"].as<double>();
    const auto lower_error_soft_limit_margin = absolute_encoder_config["lowerErrorSoftLimitMarginRad"].as<double>();
    const auto upper_error_soft_limit_margin = absolute_encoder_config["upperErrorSoftLimitMarginRad"].as<double>();

    RCLCPP_INFO(rclcpp::get_logger("hardware_builder"), "absoluteEncoder has retrieved all values");

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

std::unique_ptr<march::IncrementalEncoder> HardwareBuilder::createIncrementalEncoder(
    const YAML::Node& incremental_encoder_config, const march::MotorControllerType motor_controller_type)
{
    RCLCPP_INFO(rclcpp::get_logger("hardware_builder"), "Creating IncrementalEncoder.");
    if (!incremental_encoder_config) {
        return nullptr;
    }

    HardwareBuilder::validateRequiredKeysExist(
        incremental_encoder_config, HardwareBuilder::INCREMENTAL_ENCODER_REQUIRED_KEYS, "incrementalEncoder");

    const auto counts_per_rotation = validate_and_get_counts_per_rotation(incremental_encoder_config);
    const auto transmission = incremental_encoder_config["transmission"].as<double>();

    RCLCPP_INFO(rclcpp::get_logger("hardware_builder"), "IncrementalEncoder has retrieved all values");

    return std::make_unique<march::IncrementalEncoder>(
        /*counts_per_rotation=*/counts_per_rotation,
        /*motor_controller_type=*/motor_controller_type,
        /*direction=*/getEncoderDirection(incremental_encoder_config),
        /*transmission=*/transmission);
}

march::Encoder::Direction HardwareBuilder::getEncoderDirection(const YAML::Node& encoder_config)
{
    if (encoder_config["direction"]) {
        RCLCPP_INFO(rclcpp::get_logger("hardware_builder"), "determining encodr direction.");
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

// TODO add method for creation of torque sensor :)
std::unique_ptr<march::TorqueSensor> HardwareBuilder::createTorqueSensor(
    const YAML::Node& torque_sensor_config, const march::MotorControllerType motor_controller_type)
{
    // logger_->info("Starting to create and return torque sensors.");
    if (!torque_sensor_config) {
        return nullptr;
    }
    HardwareBuilder::validateRequiredKeysExist(
        torque_sensor_config, HardwareBuilder::TORQUE_SENSOR_REQUIRED_KEYS, "torqueSensor");

    const auto max_torque = torque_sensor_config["maxTorque"].as<float>();
    const auto average_torque = torque_sensor_config["averageTorque"].as<float>();
    return std::make_unique<march::TorqueSensor>(
        /*motor_controller_type*/ motor_controller_type,
        /*max_torque*/ max_torque,
        /*average_torque*/ average_torque);
}

std::unique_ptr<march::TemperatureGES> HardwareBuilder::createTemperatureGES(
    const YAML::Node& temperature_ges_config) const
{
    if (!temperature_ges_config) {
        return nullptr;
    }

    HardwareBuilder::validateRequiredKeysExist(
        temperature_ges_config, HardwareBuilder::TEMPERATUREGES_REQUIRED_KEYS, "temperatureges");

    const auto slave_index = temperature_ges_config["slaveIndex"].as<int>();
    const auto byte_offset = temperature_ges_config["byteOffset"].as<int>();
    return std::make_unique<march::TemperatureGES>(
        march::Slave(slave_index, pdo_interface_, sdo_interface_), byte_offset);
}


std::optional<march::PowerDistributionBoard> HardwareBuilder::createPowerDistributionBoard(
    const YAML::Node& power_distribution_board_config) const
{
    if (!power_distribution_board_config) {
        return std::nullopt;
    }
    logger_->info("Running with PowerDistributionBoard");
    HardwareBuilder::validateRequiredKeysExist(power_distribution_board_config,
        HardwareBuilder::POWER_DISTRIBUTION_BOARD_REQUIRED_KEYS, "power_distribution_board");

    const auto slave_index = power_distribution_board_config["slaveIndex"].as<int>();
    const auto byte_offset = power_distribution_board_config["byteOffset"].as<int>();
    logger_->info("Finished creating PowerDistributionBoard");
    return std::make_optional<march::PowerDistributionBoard>(
        march::Slave(slave_index, pdo_interface_, sdo_interface_), byte_offset);
}

void HardwareBuilder::validateRequiredKeysExist(
    const YAML::Node& config, const std::vector<std::string>& key_list, const std::string& object_name)
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
        // If neither is set, throw a special exception complaining about both of them.
        throw MissingKeyException("resolution' or 'countsPerRotation", "incrementalEncoder");
    }
}

// std::string convertSWFileToString(std::ifstream& sw_file)
//{
//    return std::string(std::istreambuf_iterator<char>(sw_file),
//        std::istreambuf_iterator<char>());
//}
