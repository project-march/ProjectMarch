// Copyright 2019 Project March.

#ifndef MARCH_IV_ALLOWEDROBOT_H
#define MARCH_IV_ALLOWEDROBOT_H

#include <ros/package.h>

class AllowedRobot
{
public:
  enum Value : int
  {
    march3,
    testsetup,
    pdbtestsetup
  };

  AllowedRobot() = default;
  explicit AllowedRobot(const std::string& robotName)
  {
    if (robotName == "march3")
    {
      this->value = march3;
    }
    else if (robotName == "testsetup")
    {
      this->value = testsetup;
    }
    else if (robotName == "pdbtestsetup")
    {
      this->value = pdbtestsetup;
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
    if (this->value == AllowedRobot::testsetup)
    {
      return basePath.append("/src/robots/testsetup.yaml");
    }
    else if (this->value == AllowedRobot::march3)
    {
        return basePath.append("/src/robots/march3.yaml");
    }
    else if (this->value == AllowedRobot::pdbtestsetup)
    {
        return basePath.append("/src/robots/power_distribution_board_setup.yaml");
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
