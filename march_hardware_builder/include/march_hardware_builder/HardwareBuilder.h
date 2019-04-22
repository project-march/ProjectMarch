// Copyright 2019 Project March.

#ifndef MARCH_IV_HARDWAREBUILDER_H
#define MARCH_IV_HARDWAREBUILDER_H

#include <yaml-cpp/yaml.h>

#include <march_hardware/MarchRobot.h>
#include <march_hardware/Joint.h>
#include <march_hardware/Encoder.h>
#include <march_hardware/IMotionCube.h>
#include <march_hardware/TemperatureGES.h>

#include <march_hardware_builder/AllowedRobot.h>

/**
 * @brief Creates a MarchRobot from a robot name, yamlPath, or a loaded .yaml config.
 */
class HardwareBuilder
{
private:
  const std::vector<std::string> ENCODER_REQUIRED_KEYS =
          { "resolution", "minPositionIU", "maxPositionIU", "zeroPositionIU", "safetyMarginRad" };
  const std::vector<std::string> IMOTIONCUBE_REQUIRED_KEYS = { "slaveIndex", "encoder" };
  const std::vector<std::string> TEMPERATUREGES_REQUIRED_KEYS = { "slaveIndex", "byteOffset" };
  const std::vector<std::string> JOINT_REQUIRED_KEYS = {"allowActuation"};

  /**
   * @brief Loop over all keys in the keyList and check if they exist in the config. Throws a MissingKeyException when
   *     keys are missing.
   */
  void validateRequiredKeysExist(YAML::Node config, std::vector<std::string> keyList, const std::string& objectName);

public:
  std::string yamlPath;
  YAML::Node robotConfig;

  /**
   * @brief Initialise a HardwareBuilder with a robotName enumerator.
   * @details Grabs the .yaml file associated with the robotName and loads it in the robotConfig.
   */
  explicit HardwareBuilder(AllowedRobot robot);

  /**
   * @brief Initialise a HardwareBuilder with a path to a .yaml file.
   */
  explicit HardwareBuilder(std::string yamlPath);
  HardwareBuilder();

  /**
   * @brief Create a MarchRobot from a yaml config.
   */
  march4cpp::MarchRobot createMarchRobot(YAML::Node marchRobotConfig);

  /**
   * @brief Create a MarchRobot, can only be used when a robotConfig is already loaded via the constructor.
   */
  march4cpp::MarchRobot createMarchRobot();

  march4cpp::Joint createJoint(YAML::Node jointConfig, std::string jointName);
  march4cpp::Encoder createEncoder(YAML::Node encoderConfig);
  march4cpp::IMotionCube createIMotionCube(YAML::Node iMotionCubeConfig);
  march4cpp::TemperatureGES createTemperatureGES(YAML::Node temperatureGESConfig);
};

#endif  // MARCH_IV_HARDWAREBUILDER_H
