// Copyright 2019 Project March.
#include <string>
#include <vector>

#include <ros/ros.h>

#include <march_hardware_builder/hardware_builder.h>
#include <march_hardware_builder/hardware_config_exceptions.h>

// clang-format off
const std::vector<std::string> HardwareBuilder::ENCODER_REQUIRED_KEYS =
    {
        "resolution", "minPositionIU", "maxPositionIU", "zeroPositionIU", "safetyMarginRad"
    };
const std::vector<std::string> HardwareBuilder::IMOTIONCUBE_REQUIRED_KEYS = { "slaveIndex", "encoder" };
const std::vector<std::string> HardwareBuilder::TEMPERATUREGES_REQUIRED_KEYS = { "slaveIndex", "byteOffset" };
const std::vector<std::string> HardwareBuilder::POWER_DISTRIBUTION_BOARD_REQUIRED_KEYS =
    {
        "slaveIndex", "bootShutdownOffsets", "netMonitorByteOffsets", "netDriverByteOffsets"
    };
const std::vector<std::string> HardwareBuilder::JOINT_REQUIRED_KEYS = { "allowActuation" };
// clang-format on

HardwareBuilder::HardwareBuilder(AllowedRobot robot)
    : HardwareBuilder::HardwareBuilder(robot.getFilePath())
{
}

HardwareBuilder::HardwareBuilder(const std::string& yaml_path)
  : yaml_path_(yaml_path)
  , robot_config_(YAML::LoadFile(yaml_path))
{
}

march4cpp::MarchRobot HardwareBuilder::createMarchRobot()
{
  std::string robot_name = this->robot_config_.begin()->first.as<std::string>();
  ROS_DEBUG_STREAM("Starting creation of robot " << robot_name);

  // Remove top level robot name key
  YAML::Node config = this->robot_config_[robot_name];
  std::string if_name = config["ifName"].as<std::string>();
  int cycle_time = config["ecatCycleTime"].as<int>();

  YAML::Node joint_list_config = config["joints"];

  std::vector<march4cpp::Joint> joint_list;

  for (std::size_t i = 0; i < joint_list_config.size(); i++)
  {
    YAML::Node joint_config = joint_list_config[i];
    std::string joint_name = joint_config.begin()->first.as<std::string>();
    joint_list.push_back(HardwareBuilder::createJoint(joint_config[joint_name], joint_name));
  }

  ROS_INFO_STREAM("Robot config:\n" << config);
  YAML::Node pdb_config = config["powerDistributionBoard"];
  if (pdb_config)
  {
    march4cpp::PowerDistributionBoard pdb = HardwareBuilder::createPowerDistributionBoard(pdb_config);
    return march4cpp::MarchRobot(joint_list, pdb, if_name, cycle_time);
  }
  else
  {
    ROS_INFO("powerDistributionBoard is NOT defined");
    return march4cpp::MarchRobot(joint_list, if_name, cycle_time);
  }
}

march4cpp::Joint HardwareBuilder::createJoint(const YAML::Node& joint_config, const std::string& joint_name)
{
  ROS_DEBUG("Starting creation of joint %s", joint_name.c_str());
  HardwareBuilder::validateRequiredKeysExist(joint_config, HardwareBuilder::JOINT_REQUIRED_KEYS, "joint");

  march4cpp::Joint joint;
  joint.setName(joint_name);

  bool allowActuation = joint_config["allowActuation"].as<bool>();
  joint.setAllowActuation(allowActuation);

  if (joint_config["imotioncube"])
  {
    march4cpp::IMotionCube imc = HardwareBuilder::createIMotionCube(joint_config["imotioncube"]);
    joint.setIMotionCube(imc);
  }
  else
  {
    ROS_WARN("Joint %s does not have a configuration for an IMotionCube", joint_name.c_str());
  }

  if (joint_config["actuationMode"])
  {
    std::string mode = joint_config["actuationMode"].as<std::string>();
    joint.setActuationMode(march4cpp::ActuationMode(mode));
  }
  else
  {
    joint.setActuationMode(march4cpp::ActuationMode("unknown"));
  }

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
    march4cpp::TemperatureGES ges = HardwareBuilder::createTemperatureGES(joint_config["temperatureges"]);
    joint.setTemperatureGes(ges);
  }
  else
  {
    ROS_WARN("Joint %s does not have a configuration for a TemperatureGes", joint_name.c_str());
  }
  return joint;
}

march4cpp::IMotionCube HardwareBuilder::createIMotionCube(const YAML::Node& imc_config)
{
  HardwareBuilder::validateRequiredKeysExist(imc_config, HardwareBuilder::IMOTIONCUBE_REQUIRED_KEYS, "imotioncube");

  YAML::Node encoder_config = imc_config["encoder"];
  int slave_index = imc_config["slaveIndex"].as<int>();
  return march4cpp::IMotionCube(slave_index, HardwareBuilder::createEncoder(encoder_config));
}

march4cpp::Encoder HardwareBuilder::createEncoder(const YAML::Node& encoder_config)
{
  HardwareBuilder::validateRequiredKeysExist(encoder_config, HardwareBuilder::ENCODER_REQUIRED_KEYS, "encoder");

  int resolution = encoder_config["resolution"].as<int>();
  int min_position = encoder_config["minPositionIU"].as<int>();
  int max_position = encoder_config["maxPositionIU"].as<int>();
  int zero_position = encoder_config["zeroPositionIU"].as<int>();
  float safety_margin = encoder_config["safetyMarginRad"].as<float>();
  return march4cpp::Encoder(resolution, min_position, max_position, zero_position, safety_margin);
}

march4cpp::TemperatureGES HardwareBuilder::createTemperatureGES(const YAML::Node& temperature_ges_config)
{
  HardwareBuilder::validateRequiredKeysExist(temperature_ges_config,
                                  HardwareBuilder::TEMPERATUREGES_REQUIRED_KEYS,
                                  "temperatureges");

  int slave_index = temperature_ges_config["slaveIndex"].as<int>();
  int byte_offset = temperature_ges_config["byteOffset"].as<int>();
  return march4cpp::TemperatureGES(slave_index, byte_offset);
}

march4cpp::PowerDistributionBoard HardwareBuilder::createPowerDistributionBoard(const YAML::Node& pdb)
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
      net_monitor_byte_offsets["highVoltageEnabled"].as<int>(),
      net_monitor_byte_offsets["highVoltageState"].as<int>());

  NetDriverOffsets net_driver_offsets = NetDriverOffsets(net_driver_byte_offsets["lowVoltageNetOnOff"].as<int>(),
                                                       net_driver_byte_offsets["highVoltageNetOnOff"].as<int>(),
                                                       net_driver_byte_offsets["highVoltageNetEnableDisable"].as<int>());

  BootShutdownOffsets boot_shutdown_offsets =
      BootShutdownOffsets(boot_shutdown_byte_offsets["masterOk"].as<int>(), boot_shutdown_byte_offsets["shutdown"].as<int>(),
                          boot_shutdown_byte_offsets["shutdownAllowed"].as<int>());

  return march4cpp::PowerDistributionBoard(slave_index, net_monitor_offsets, net_driver_offsets, boot_shutdown_offsets);
}

void HardwareBuilder::validateRequiredKeysExist(const YAML::Node& config,
                                                const std::vector<std::string>& key_list,
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
