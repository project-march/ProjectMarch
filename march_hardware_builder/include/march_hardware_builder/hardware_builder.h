// Copyright 2019 Project March.
#ifndef MARCH_HARDWARE_BUILDER_HARDWAREBUILDER_H
#define MARCH_HARDWARE_BUILDER_HARDWAREBUILDER_H
#include <string>
#include <vector>
#include <yaml-cpp/yaml.h>

#include <march_hardware/Encoder.h>
#include <march_hardware/IMotionCube.h>
#include <march_hardware/Joint.h>
#include <march_hardware/MarchRobot.h>
#include <march_hardware/PowerDistributionBoard.h>
#include <march_hardware/TemperatureGES.h>

#include "march_hardware_builder/allowed_robot.h"

/**
 * @brief Creates a MarchRobot from a robot name, yamlPath, or a loaded .yaml
 * config.
 */
class HardwareBuilder
{
public:
  /**
   * @brief Initialise a HardwareBuilder with a robotName enumerator.
   * @details Grabs the .yaml file associated with the robotName and loads it in
   * the robotConfig.
   */
  explicit HardwareBuilder(AllowedRobot robot);

  /**
   * @brief Initialise a HardwareBuilder with a path to a .yaml file.
   */
  explicit HardwareBuilder(const std::string& yaml_path);

  /**
   * @brief Create a MarchRobot, can only be used when a robotConfig is already
   * loaded via the constructor.
   */
  march4cpp::MarchRobot createMarchRobot();

  /**
   * @brief Loop over all keys in the keyList and check if they exist in the
   * config. Throws a MissingKeyException when
   *     keys are missing.
   */
  static void validateRequiredKeysExist(const YAML::Node& config, const std::vector<std::string>& key_list,
                                        const std::string& object_name);

  static march4cpp::Joint createJoint(const YAML::Node& joint_config, const std::string& joint_name);
  static march4cpp::Encoder createEncoder(const YAML::Node& encoder_config);
  static march4cpp::IMotionCube createIMotionCube(const YAML::Node& imc_config);
  static march4cpp::TemperatureGES createTemperatureGES(const YAML::Node& temperature_ges_config);
  static march4cpp::PowerDistributionBoard
  createPowerDistributionBoard(const YAML::Node& power_distribution_board_config);

  static const std::vector<std::string> ENCODER_REQUIRED_KEYS;
  static const std::vector<std::string> IMOTIONCUBE_REQUIRED_KEYS;
  static const std::vector<std::string> TEMPERATUREGES_REQUIRED_KEYS;
  static const std::vector<std::string> POWER_DISTRIBUTION_BOARD_REQUIRED_KEYS;
  static const std::vector<std::string> JOINT_REQUIRED_KEYS;

private:
  std::string yaml_path_;
  YAML::Node robot_config_;
};

#endif  // MARCH_HARDWARE_BUILDER_HARDWAREBUILDER_H
