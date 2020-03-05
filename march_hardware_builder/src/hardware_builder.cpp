// Copyright 2019 Project March.
#include "march_hardware_builder/hardware_builder.h"
#include "march_hardware_builder/hardware_config_exceptions.h"

#include <string>
#include <vector>

#include <ros/ros.h>

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
const std::vector<std::string> HardwareBuilder::JOINT_REQUIRED_KEYS = { "allowActuation", "imotioncube" };
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

march::MarchRobot HardwareBuilder::createMarchRobot()
{
  this->initUrdf();

  std::string robot_name = this->robot_config_.begin()->first.as<std::string>();
  ROS_DEBUG_STREAM("Starting creation of robot " << robot_name);

  // Remove top level robot name key
  YAML::Node config = this->robot_config_[robot_name];
  std::string if_name = config["ifName"].as<std::string>();
  int cycle_time = config["ecatCycleTime"].as<int>();

  YAML::Node joint_list_config = config["joints"];

  std::vector<march::Joint> joint_list;

  for (const YAML::Node& joint_config : joint_list_config)
  {
    std::string joint_name = joint_config.begin()->first.as<std::string>();
    joint_list.push_back(
        HardwareBuilder::createJoint(joint_config[joint_name], joint_name, this->urdf_.getJoint(joint_name)));
  }

  ROS_INFO_STREAM("Robot config:\n" << config);
  YAML::Node pdb_config = config["powerDistributionBoard"];
  if (pdb_config)
  {
    march::PowerDistributionBoard pdb = HardwareBuilder::createPowerDistributionBoard(pdb_config);
    return { joint_list, this->urdf_, pdb, if_name, cycle_time };
  }
  else
  {
    ROS_INFO("powerDistributionBoard is NOT defined");
    return { joint_list, this->urdf_, if_name, cycle_time };
  }
}

march::Joint HardwareBuilder::createJoint(const YAML::Node& joint_config, const std::string& joint_name,
                                          const urdf::JointConstSharedPtr& urdf_joint)
{
  ROS_DEBUG("Starting creation of joint %s", joint_name.c_str());
  HardwareBuilder::validateRequiredKeysExist(joint_config, HardwareBuilder::JOINT_REQUIRED_KEYS, "joint");

  march::ActuationMode mode;
  if (joint_config["actuationMode"])
  {
    mode = march::ActuationMode(joint_config["actuationMode"].as<std::string>());
  }

  march::IMotionCube imc = HardwareBuilder::createIMotionCube(joint_config["imotioncube"], mode, urdf_joint);

  march::Joint joint(imc);
  joint.setName(joint_name);

  bool allowActuation = joint_config["allowActuation"].as<bool>();
  joint.setAllowActuation(allowActuation);

  if (joint_config["netNumber"])
  {
    joint.setNetNumber(joint_config["netNumber"].as<int>());
  }
  else
  {
    ROS_WARN("Joint %s does not have a netNumber", joint_name.c_str());
    joint.setNetNumber(-1);
  }

  if (joint_config["temperatureges"])
  {
    march::TemperatureGES ges = HardwareBuilder::createTemperatureGES(joint_config["temperatureges"]);
    joint.setTemperatureGes(ges);
  }
  else
  {
    ROS_WARN("Joint %s does not have a configuration for a TemperatureGes", joint_name.c_str());
  }
  return joint;
}

march::IMotionCube HardwareBuilder::createIMotionCube(const YAML::Node& imc_config, march::ActuationMode mode,
                                                      const urdf::JointConstSharedPtr& urdf_joint)
{
  HardwareBuilder::validateRequiredKeysExist(imc_config, HardwareBuilder::IMOTIONCUBE_REQUIRED_KEYS, "imotioncube");

  YAML::Node incremental_encoder_config = imc_config["incrementalEncoder"];
  YAML::Node absolute_encoder_config = imc_config["absoluteEncoder"];
  int slave_index = imc_config["slaveIndex"].as<int>();
  return { slave_index, HardwareBuilder::createAbsoluteEncoder(absolute_encoder_config, urdf_joint),
           HardwareBuilder::createIncrementalEncoder(incremental_encoder_config), mode };
}

march::AbsoluteEncoder HardwareBuilder::createAbsoluteEncoder(const YAML::Node& absolute_encoder_config,
                                                              const urdf::JointConstSharedPtr& urdf_joint)
{
  HardwareBuilder::validateRequiredKeysExist(absolute_encoder_config, HardwareBuilder::ABSOLUTE_ENCODER_REQUIRED_KEYS,
                                             "absoluteEncoder");

  auto resolution = absolute_encoder_config["resolution"].as<size_t>();
  auto min_position = absolute_encoder_config["minPositionIU"].as<int32_t>();
  auto max_position = absolute_encoder_config["maxPositionIU"].as<int32_t>();

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

  return { resolution,       min_position,    max_position, urdf_joint->limits->lower, urdf_joint->limits->upper,
           soft_lower_limit, soft_upper_limit };
}

march::IncrementalEncoder HardwareBuilder::createIncrementalEncoder(const YAML::Node& incremental_encoder_config)
{
  HardwareBuilder::validateRequiredKeysExist(incremental_encoder_config,
                                             HardwareBuilder::INCREMENTAL_ENCODER_REQUIRED_KEYS, "incrementalEncoder");

  auto resolution = incremental_encoder_config["resolution"].as<size_t>();
  auto transmission = incremental_encoder_config["transmission"].as<double>();
  return { resolution, transmission };
}

march::TemperatureGES HardwareBuilder::createTemperatureGES(const YAML::Node& temperature_ges_config)
{
  HardwareBuilder::validateRequiredKeysExist(temperature_ges_config, HardwareBuilder::TEMPERATUREGES_REQUIRED_KEYS,
                                             "temperatureges");

  int slave_index = temperature_ges_config["slaveIndex"].as<int>();
  int byte_offset = temperature_ges_config["byteOffset"].as<int>();
  return { slave_index, byte_offset };
}

march::PowerDistributionBoard HardwareBuilder::createPowerDistributionBoard(const YAML::Node& pdb)
{
  HardwareBuilder::validateRequiredKeysExist(pdb, HardwareBuilder::POWER_DISTRIBUTION_BOARD_REQUIRED_KEYS,
                                             "powerdistributionboard");

  int slave_index = pdb["slaveIndex"].as<int>();
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

  NetDriverOffsets net_driver_offsets = NetDriverOffsets(
      net_driver_byte_offsets["lowVoltageNetOnOff"].as<int>(), net_driver_byte_offsets["highVoltageNetOnOff"].as<int>(),
      net_driver_byte_offsets["highVoltageNetEnableDisable"].as<int>());

  BootShutdownOffsets boot_shutdown_offsets = BootShutdownOffsets(
      boot_shutdown_byte_offsets["masterOk"].as<int>(), boot_shutdown_byte_offsets["shutdown"].as<int>(),
      boot_shutdown_byte_offsets["shutdownAllowed"].as<int>());

  return { slave_index, net_monitor_offsets, net_driver_offsets, boot_shutdown_offsets };
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
      throw HardwareConfigException("Failed to load urdf from parameter server");
    }
    this->init_urdf_ = false;
  }
}
