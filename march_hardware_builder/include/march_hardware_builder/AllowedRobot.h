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
    testsetup_rot,
    testsetup_lin,
    pdb,
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
    else if (robotName == "testsetup_rot")
    {
      this->value = testsetup_rot;
    }
    else if (robotName == "testsetup_lin")
    {
      this->value = testsetup_lin;
    }
    else if (robotName == "pdb")
    {
      this->value = pdb;
    }
    else
    {
      ROS_ASSERT_MSG(false, "Unknown robot %s", robotName.c_str());
      this->value = AllowedRobot::march4;
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
    else if (this->value == AllowedRobot::testsetup_rot)
    {
      return basePath.append("/src/robots/test_setup_rot.yaml");
    }
    else if (this->value == AllowedRobot::testsetup_lin)
    {
      return basePath.append("/src/robots/test_setup_lin.yaml");
    }
    else if (this->value == AllowedRobot::pdb)
    {
      return basePath.append("/src/robots/pdb.yaml");
    }
    ROS_ERROR("Robotname not implemented. Using march4.yaml...");
    return basePath.append("/src/robots/march4.yaml");
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
