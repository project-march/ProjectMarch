//
// Created by projectmarch on 28-3-19.
//

#ifndef MARCH_IV_HARDWAREBUILDER_H
#define MARCH_IV_HARDWAREBUILDER_H

#include <yaml-cpp/yaml.h>

#include <march_hardware/MarchRobot.h>
#include <march_hardware/Joint.h>
#include <march_hardware/Encoder.h>
#include <march_hardware/IMotionCube.h>
#include <march_hardware/TemperatureGES.h>

class HardwareBuilder
{
private:
  const std::vector<std::string> ENCODER_REQUIRED_KEYS = { "resolution", "minPositionIU", "maxPositionIU",
                                                           "zeroPositionIU", "safetyMarginRad" };
  const std::vector<std::string> IMOTIONCUBE_REQUIRED_KEYS = { "slaveIndex", "encoder" };
  const std::vector<std::string> TEMPERATUREGES_REQUIRED_KEYS = { "slaveIndex", "byteOffset" };
  const std::vector<std::string> JOINT_REQUIRED_KEYS = {};

public:
  HardwareBuilder();

  march4cpp::MarchRobot createMarchRobot(YAML::Node marchRobotConfig);
  march4cpp::Joint createJoint(YAML::Node jointConfig, std::string jointName);
  march4cpp::Encoder createEncoder(YAML::Node encoderConfig);
  march4cpp::IMotionCube createIMotionCube(YAML::Node iMotionCubeConfig);
  march4cpp::TemperatureGES createTemperatureGES(YAML::Node temperatureGESConfig);

  void validateRequiredKeysExist(YAML::Node config, std::vector<std::string> keyList, const std::string& objectName);
};

#endif  // MARCH_IV_HARDWAREBUILDER_H
