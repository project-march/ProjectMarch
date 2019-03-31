// Copyright 2019 Project March.

#include <march_hardware_builder/HardwareBuilder.h>
#include <march_hardware_builder/HardwareConfigExceptions.h>

HardwareBuilder::HardwareBuilder()
{
}

march4cpp::Encoder HardwareBuilder::createEncoder(YAML::Node EncoderConfig)
{
  this->validateRequiredKeysExist(EncoderConfig, this->ENCODER_REQUIRED_KEYS, "encoder");

  int resolution = EncoderConfig["resolution"].as<int>();
  int minPositionIU = EncoderConfig["minPositionIU"].as<int>();
  int maxPositionIU = EncoderConfig["maxPositionIU"].as<int>();
  int zeroPositionIU = EncoderConfig["zeroPositionIU"].as<int>();
  auto safetyMarginRad = EncoderConfig["safetyMarginRad"].as<float>();
  return march4cpp::Encoder(resolution, minPositionIU, maxPositionIU, zeroPositionIU, safetyMarginRad);
}

march4cpp::MarchRobot HardwareBuilder::createMarchRobot(YAML::Node marchRobotConfig) {
    std::vector<march4cpp::Joint> jointList;

//    return march4cpp::MarchRobot();
}


march4cpp::Joint HardwareBuilder::createJoint(YAML::Node jointConfig, std::string jointName)
{
    march4cpp::IMotionCube imc;
    march4cpp::TemperatureGES temperatureGes;

    this->validateRequiredKeysExist(jointConfig, this->JOINT_REQUIRED_KEYS, "joint");

    if (jointConfig["imotioncube"].Type() == YAML::NodeType::Undefined)
    {
        ROS_WARN("Joint %s does not have a configuration for an IMotionCube", jointName.c_str());
    } else {
        imc = this->createIMotionCube(jointConfig["imotioncube"]);
    }

    if (jointConfig["temperatureges"].Type() == YAML::NodeType::Undefined)
    {
        ROS_WARN("Joint %s does not have a configuration for a TemperatureGes", jointName.c_str());
    } else {
        temperatureGes = this->createTemperatureGES(jointConfig["temperatureges"]);
    }

    return march4cpp::Joint(jointName, temperatureGes, imc);
}


march4cpp::IMotionCube HardwareBuilder::createIMotionCube(YAML::Node iMotionCubeConfig)
{
  this->validateRequiredKeysExist(iMotionCubeConfig, this->IMOTIONCUBE_REQUIRED_KEYS, "imotioncube");

  YAML::Node encoderConfig = iMotionCubeConfig["encoder"];
  int slaveIndex = iMotionCubeConfig["slaveIndex"].as<int>();
  return march4cpp::IMotionCube(slaveIndex, this->createEncoder(encoderConfig));
}

march4cpp::TemperatureGES HardwareBuilder::createTemperatureGES(YAML::Node temperatureGESConfig)
{
  this->validateRequiredKeysExist(temperatureGESConfig, this->TEMPERATUREGES_REQUIRED_KEYS, "temperatureges");

  int slaveIndex = temperatureGESConfig["slaveIndex"].as<int>();
  int byteOffset = temperatureGESConfig["byteOffset"].as<int>();
  return march4cpp::TemperatureGES(slaveIndex, byteOffset);
}

void HardwareBuilder::validateRequiredKeysExist(YAML::Node config, std::vector<std::string> keyList,
                                                const std::string& objectName)
{
  for (std::vector<std::string>::size_type i = 0; i != keyList.size(); i++)
  {
    if (config[keyList.at(i)].Type() == YAML::NodeType::Undefined)
    {
      throw MissingKeyException(keyList.at(i), objectName);
    }
  }
}
