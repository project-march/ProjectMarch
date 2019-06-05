// Copyright 2019 Project March.

#ifndef MARCH_IV_ALLOWEDROBOT_H
#define MARCH_IV_ALLOWEDROBOT_H

#include <ros/package.h>

class AllowedRobot
{
public:
  enum Value : int
  {
    march4,
    march3,
    testsetup,
  };

  AllowedRobot() = default;
  explicit AllowedRobot(const std::string& robotName)
  {
    if (robotName == "march4")
    {
      this->value = march4;
    }
    else if (robotName == "march3")
    {
        this->value = march4;
    }
    else if (robotName == "testsetup")
    {
      this->value = testsetup;
    }
    else
    {
      ROS_ASSERT_MSG(false, "Unknown robot %s", robotName.c_str());
      this->value = AllowedRobot::testsetup;
    }
  }

  std::string getFilePath()
  {
    std::string basePath = ros::package::getPath("march_hardware_builder");
    if (this->value == AllowedRobot::march4)
    {
      return basePath.append("/src/robots/march4.yaml");
    }
    else if (this->value == AllowedRobot::march3)
    {
      return basePath.append("/src/robots/march3.yaml");
    }
    else if (this->value == AllowedRobot::testsetup)
    {
        return basePath.append("/src/robots/testsetup.yaml");
    }
    ROS_ERROR("Robotname not implemented. Using march3.yaml...");
    return basePath.append("/src/robots/march3.yaml");
  }

  constexpr AllowedRobot(Value aAllowedRobot) : value(aAllowedRobot)
  {
  }

  bool operator==(AllowedRobot a) const
  {
    return value == a.value;
  }
  bool operator!=(AllowedRobot a) const
  {
    return value != a.value;
  }

private:
  Value value;
};

#endif  // MARCH_IV_ALLOWEDROBOT_H
