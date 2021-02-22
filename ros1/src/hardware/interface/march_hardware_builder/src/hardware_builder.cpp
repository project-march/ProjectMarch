// Copyright 2019 Project March.
#include "march_hardware_builder/hardware_builder.h"
#include "march_hardware_builder/hardware_config_exceptions.h"

#include <algorithm>
#include <memory>
#include <string>
#include <utility>
#include <vector>
#include <fstream>

#include <ros/ros.h>

#include <march_hardware/error/error_type.h>
#include <march_hardware/ethercat/pdo_interface.h>
#include <march_hardware/ethercat/sdo_interface.h>
#include <march_hardware/error/hardware_exception.h>
#include <march_hardware/pressure_sole/pressure_sole.h>

// clang-format off
const std::vector<std::string> HardwareBuilder::ABSOLUTE_ENCODER_REQUIRED_KEYS =
    {
        "resolution", "minPositionIU", "maxPositionIU"
    };
const std::vector<std::string> HardwareBuilder::INCREMENTAL_ENCODER_REQUIRED_KEYS = { "resolution", "transmission" };
const std::vector<std::string> HardwareBuilder::IMOTIONCUBE_REQUIRED_KEYS =
    {
        "slaveIndex", "incrementalEncoder", "absoluteEncoder"
    };
const std::vector<std::string> HardwareBuilder::TEMPERATUREGES_REQUIRED_KEYS = { "slaveIndex", "byteOffset" };
const std::vector<std::string> HardwareBuilder::POWER_DISTRIBUTION_BOARD_REQUIRED_KEYS =
    {
        "slaveIndex", "bootShutdownOffsets", "netMonitorByteOffsets", "netDriverByteOffsets"
    };
const std::vector<std::string> HardwareBuilder::JOINT_REQUIRED_KEYS = { "allowActuation" };
const std::vector<std::string> HardwareBuilder::PRESSURE_SOLE_REQUIRED_KEYS = { "slaveIndex", "byteOffset", "side" };
// clang-format on

HardwareBuilder::HardwareBuilder(AllowedRobot robot) : HardwareBuilder(robot.getFilePath())
{
}

HardwareBuilder::HardwareBuilder(AllowedRobot robot, urdf::Model urdf)
  : robot_config_(YAML::LoadFile(robot.getFilePath())), urdf_(std::move(urdf)), init_urdf_(false)
{
}

HardwareBuilder::HardwareBuilder(const std::string& yaml_path) : robot_config_(YAML::LoadFile(yaml_path))
{
}

HardwareBuilder::HardwareBuilder(const std::string& yaml_path, urdf::Model urdf)
  : robot_config_(YAML::LoadFile(yaml_path)), urdf_(std::move(urdf)), init_urdf_(false)
{
}

std::unique_ptr<march::MarchRobot> HardwareBuilder::createMarchRobot()
{
  this->initUrdf();
  auto pdo_interface = march::PdoInterfaceImpl::create();
  auto sdo_interface = march::SdoInterfaceImpl::create();

  const auto robot_name = this->robot_config_.begin()->first.as<std::string>();
  ROS_DEBUG_STREAM("Starting creation of robot " << robot_name);

  // Remove top level robot name key
  YAML::Node config = this->robot_config_[robot_name];
  const auto if_name = config["ifName"].as<std::string>();
  const auto cycle_time = config["ecatCycleTime"].as<int>();
  const auto slave_timeout = config["ecatSlaveTimeout"].as<int>();

  std::vector<march::Joint> joints = this->createJoints(config["joints"], pdo_interface, sdo_interface);

  ROS_INFO_STREAM("Robot config:\n" << config);
  YAML::Node pdb_config = config["powerDistributionBoard"];
  auto pdb = HardwareBuilder::createPowerDistributionBoard(pdb_config, pdo_interface, sdo_interface);
  auto pressure_soles = createPressureSoles(config["pressure_soles"], pdo_interface, sdo_interface);
  return std::make_unique<march::MarchRobot>(std::move(joints), this->urdf_, std::move(pdb), std::move(pressure_soles),
                                             if_name, cycle_time, slave_timeout);
}

march::Joint HardwareBuilder::createJoint(const YAML::Node& joint_config, const std::string& joint_name,
                                          const urdf::JointConstSharedPtr& urdf_joint,
                                          march::PdoInterfacePtr pdo_interface, march::SdoInterfacePtr sdo_interface)
{
  ROS_DEBUG("Starting creation of joint %s", joint_name.c_str());
  if (!urdf_joint)
  {
    throw march::error::HardwareException(march::error::ErrorType::MISSING_URDF_JOINT,
                                          "No URDF joint given for joint " + joint_name);
  }
  HardwareBuilder::validateRequiredKeysExist(joint_config, HardwareBuilder::JOINT_REQUIRED_KEYS, "joint");

  auto net_number = -1;
  if (joint_config["netNumber"])
  {
    net_number = joint_config["netNumber"].as<int>();
  }
  else
  {
    ROS_WARN("Joint %s does not have a netNumber", joint_name.c_str());
  }

  const auto allow_actuation = joint_config["allowActuation"].as<bool>();

  march::ActuationMode mode;
  if (joint_config["actuationMode"])
  {
    mode = march::ActuationMode(joint_config["actuationMode"].as<std::string>());
  }

  auto imc =
      HardwareBuilder::createIMotionCube(joint_config["imotioncube"], mode, urdf_joint, pdo_interface, sdo_interface);
  if (!imc)
  {
    ROS_WARN("Joint %s does not have a configuration for an IMotionCube", joint_name.c_str());
  }

  auto ges = HardwareBuilder::createTemperatureGES(joint_config["temperatureges"], pdo_interface, sdo_interface);
  if (!ges)
  {
    ROS_WARN("Joint %s does not have a configuration for a TemperatureGes", joint_name.c_str());
  }
  return { joint_name, net_number, allow_actuation, std::move(imc), std::move(ges) };
}

std::unique_ptr<march::IMotionCube> HardwareBuilder::createIMotionCube(const YAML::Node& imc_config,
                                                                       march::ActuationMode mode,
                                                                       const urdf::JointConstSharedPtr& urdf_joint,
                                                                       march::PdoInterfacePtr pdo_interface,
                                                                       march::SdoInterfacePtr sdo_interface)
{
  if (!imc_config || !urdf_joint)
  {
    return nullptr;
  }

  HardwareBuilder::validateRequiredKeysExist(imc_config, HardwareBuilder::IMOTIONCUBE_REQUIRED_KEYS, "imotioncube");

  YAML::Node incremental_encoder_config = imc_config["incrementalEncoder"];
  YAML::Node absolute_encoder_config = imc_config["absoluteEncoder"];
  int slave_index = imc_config["slaveIndex"].as<int>();

  std::ifstream imc_setup_data;
  imc_setup_data.open(ros::package::getPath("march_hardware").append("/config/sw_files/" + urdf_joint->name + ".sw"));
  std::string setup = convertSWFileToString(imc_setup_data);
  return std::make_unique<march::IMotionCube>(
      march::Slave(slave_index, pdo_interface, sdo_interface),
      HardwareBuilder::createAbsoluteEncoder(absolute_encoder_config, urdf_joint),
      HardwareBuilder::createIncrementalEncoder(incremental_encoder_config), setup, mode);
}

std::unique_ptr<march::AbsoluteEncoder> HardwareBuilder::createAbsoluteEncoder(
    const YAML::Node& absolute_encoder_config, const urdf::JointConstSharedPtr& urdf_joint)
{
  if (!absolute_encoder_config || !urdf_joint)
  {
    return nullptr;
  }

  HardwareBuilder::validateRequiredKeysExist(absolute_encoder_config, HardwareBuilder::ABSOLUTE_ENCODER_REQUIRED_KEYS,
                                             "absoluteEncoder");

  const auto resolution = absolute_encoder_config["resolution"].as<size_t>();
  const auto min_position = absolute_encoder_config["minPositionIU"].as<int32_t>();
  const auto max_position = absolute_encoder_config["maxPositionIU"].as<int32_t>();

  double soft_lower_limit;
  double soft_upper_limit;
  if (urdf_joint->safety)
  {
    soft_lower_limit = urdf_joint->safety->soft_lower_limit;
    soft_upper_limit = urdf_joint->safety->soft_upper_limit;
  }
  else
  {
    ROS_WARN("URDF joint %s has no defined soft limits, so using hard limits as soft limits", urdf_joint->name.c_str());
    soft_lower_limit = urdf_joint->limits->lower;
    soft_upper_limit = urdf_joint->limits->upper;
  }

  return std::make_unique<march::AbsoluteEncoder>(resolution, min_position, max_position, urdf_joint->limits->lower,
                                                  urdf_joint->limits->upper, soft_lower_limit, soft_upper_limit);
}

std::unique_ptr<march::IncrementalEncoder>
HardwareBuilder::createIncrementalEncoder(const YAML::Node& incremental_encoder_config)
{
  if (!incremental_encoder_config)
  {
    return nullptr;
  }

  HardwareBuilder::validateRequiredKeysExist(incremental_encoder_config,
                                             HardwareBuilder::INCREMENTAL_ENCODER_REQUIRED_KEYS, "incrementalEncoder");

  const auto resolution = incremental_encoder_config["resolution"].as<size_t>();
  const auto transmission = incremental_encoder_config["transmission"].as<double>();
  return std::make_unique<march::IncrementalEncoder>(resolution, transmission);
}

std::unique_ptr<march::TemperatureGES> HardwareBuilder::createTemperatureGES(const YAML::Node& temperature_ges_config,
                                                                             march::PdoInterfacePtr pdo_interface,
                                                                             march::SdoInterfacePtr sdo_interface)
{
  if (!temperature_ges_config)
  {
    return nullptr;
  }

  HardwareBuilder::validateRequiredKeysExist(temperature_ges_config, HardwareBuilder::TEMPERATUREGES_REQUIRED_KEYS,
                                             "temperatureges");

  const auto slave_index = temperature_ges_config["slaveIndex"].as<int>();
  const auto byte_offset = temperature_ges_config["byteOffset"].as<int>();
  return std::make_unique<march::TemperatureGES>(march::Slave(slave_index, pdo_interface, sdo_interface), byte_offset);
}

std::unique_ptr<march::PowerDistributionBoard> HardwareBuilder::createPowerDistributionBoard(
    const YAML::Node& pdb, march::PdoInterfacePtr pdo_interface, march::SdoInterfacePtr sdo_interface)
{
  if (!pdb)
  {
    return nullptr;
  }

  HardwareBuilder::validateRequiredKeysExist(pdb, HardwareBuilder::POWER_DISTRIBUTION_BOARD_REQUIRED_KEYS,
                                             "powerdistributionboard");

  const auto slave_index = pdb["slaveIndex"].as<int>();
  YAML::Node net_monitor_byte_offsets = pdb["netMonitorByteOffsets"];
  YAML::Node net_driver_byte_offsets = pdb["netDriverByteOffsets"];
  YAML::Node boot_shutdown_byte_offsets = pdb["bootShutdownOffsets"];

  NetMonitorOffsets net_monitor_offsets = NetMonitorOffsets(
      net_monitor_byte_offsets["powerDistributionBoardCurrent"].as<int>(),
      net_monitor_byte_offsets["lowVoltageNet1Current"].as<int>(),
      net_monitor_byte_offsets["lowVoltageNet2Current"].as<int>(),
      net_monitor_byte_offsets["highVoltageNetCurrent"].as<int>(),
      net_monitor_byte_offsets["lowVoltageState"].as<int>(),
      net_monitor_byte_offsets["highVoltageOvercurrentTrigger"].as<int>(),
      net_monitor_byte_offsets["highVoltageEnabled"].as<int>(), net_monitor_byte_offsets["highVoltageState"].as<int>());

  NetDriverOffsets net_driver_offsets = NetDriverOffsets(net_driver_byte_offsets["lowVoltageNetOnOff"].as<int>(),
                                                         net_driver_byte_offsets["highVoltageNetOnOff"].as<int>(),
                                                         net_driver_byte_offsets["allHighVoltageOnOff"].as<int>());

  BootShutdownOffsets boot_shutdown_offsets = BootShutdownOffsets(
      boot_shutdown_byte_offsets["masterOk"].as<int>(), boot_shutdown_byte_offsets["shutdown"].as<int>(),
      boot_shutdown_byte_offsets["shutdownAllowed"].as<int>());

  return std::make_unique<march::PowerDistributionBoard>(march::Slave(slave_index, pdo_interface, sdo_interface),
                                                         net_monitor_offsets, net_driver_offsets,
                                                         boot_shutdown_offsets);
}

void HardwareBuilder::validateRequiredKeysExist(const YAML::Node& config, const std::vector<std::string>& key_list,
                                                const std::string& object_name)
{
  for (const std::string& key : key_list)
  {
    if (!config[key])
    {
      throw MissingKeyException(key, object_name);
    }
  }
}

void HardwareBuilder::initUrdf()
{
  if (this->init_urdf_)
  {
    if (!this->urdf_.initParam("/robot_description"))
    {
      throw march::error::HardwareException(march::error::ErrorType::INIT_URDF_FAILED);
    }
    this->init_urdf_ = false;
  }
}

std::vector<march::Joint> HardwareBuilder::createJoints(const YAML::Node& joints_config,
                                                        march::PdoInterfacePtr pdo_interface,
                                                        march::SdoInterfacePtr sdo_interface) const
{
  std::vector<march::Joint> joints;
  for (const YAML::Node& joint_config : joints_config)
  {
    const auto joint_name = joint_config.begin()->first.as<std::string>();
    const auto urdf_joint = this->urdf_.getJoint(joint_name);
    if (urdf_joint->type == urdf::Joint::FIXED)
    {
      ROS_WARN("Joint %s is fixed in the URDF, but defined in the robot yaml", joint_name.c_str());
    }
    joints.push_back(
        HardwareBuilder::createJoint(joint_config[joint_name], joint_name, urdf_joint, pdo_interface, sdo_interface));
  }

  for (const auto& urdf_joint : this->urdf_.joints_)
  {
    if (urdf_joint.second->type != urdf::Joint::FIXED)
    {
      auto equals_joint_name = [&](const auto& joint) { return joint.getName() == urdf_joint.first; };
      auto result = std::find_if(joints.begin(), joints.end(), equals_joint_name);
      if (result == joints.end())
      {
        ROS_WARN("Joint %s in URDF is not defined in robot yaml", urdf_joint.first.c_str());
      }
    }
  }

  joints.shrink_to_fit();
  return joints;
}

std::vector<march::PressureSole> HardwareBuilder::createPressureSoles(const YAML::Node& pressure_soles_config,
                                                                      march::PdoInterfacePtr pdo_interface,
                                                                      march::SdoInterfacePtr sdo_interface)
{
  std::vector<march::PressureSole> pressure_soles;
  if (!pressure_soles_config)
  {
    return pressure_soles;
  }
  for (const YAML::Node& pressure_sole_config : pressure_soles_config)
  {
    pressure_soles.push_back(
        HardwareBuilder::createPressureSole(pressure_sole_config[pressure_sole_config.begin()->first.as<std::string>()],
                                            pdo_interface, sdo_interface));
  }
  return pressure_soles;
}

march::PressureSole HardwareBuilder::createPressureSole(const YAML::Node& pressure_sole_config,
                                                         march::PdoInterfacePtr pdo_interface,
                                                         march::SdoInterfacePtr sdo_interface)
{
  HardwareBuilder::validateRequiredKeysExist(pressure_sole_config, HardwareBuilder::PRESSURE_SOLE_REQUIRED_KEYS,
                                             "pressure_sole");

  const auto slave_index = pressure_sole_config["slaveIndex"].as<int>();
  const auto byte_offset = pressure_sole_config["byteOffset"].as<int>();
  const auto side = pressure_sole_config["side"].as<std::string>();
  return march::PressureSole(march::Slave(slave_index, pdo_interface, sdo_interface), byte_offset, side);
}


std::string convertSWFileToString(std::ifstream& sw_file)
{
  return std::string(std::istreambuf_iterator<char>(sw_file), std::istreambuf_iterator<char>());
}
