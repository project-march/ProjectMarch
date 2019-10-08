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
    testjoint_rotational,
    testjoint_linear,
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
        this->value = march3;
    }
    else if (robotName == "testjoint_rotational")
    {
      this->value = testjoint_rotational;
    }
    else if (robotName == "testjoint_linear")
    {
      this->value = testjoint_linear;
    }
    else if (robotName == "pdb")
    {
      this->value = pdb;
    }
    else
    {
      ROS_ASSERT_MSG(false, "Unknown robot %s", robotName.c_str());
      this->value = AllowedRobot::testjoint_rotational;
    }
  }

  std::string getFilePath()
  {
    std::string basePath = ros::package::getPath("march_hardware_builder");
    if (this->value == AllowedRobot::march4)
    {
      return basePath.append("/robots/march4.yaml");
    }
    else if (this->value == AllowedRobot::march3)
    {
      return basePath.append("/robots/march3.yaml");
    }
    else if (this->value == AllowedRobot::testjoint_rotational)
    {
      return basePath.append("/robots/test_joint_rotational.yaml");
    }
    else if (this->value == AllowedRobot::testjoint_linear)
    {
      return basePath.append("/robots/test_joint_linear.yaml");
    }
    else if (this->value == AllowedRobot::pdb)
    {
      return basePath.append("/robots/pdb.yaml");
    }
    ROS_ERROR("Robotname not implemented. Using test_joint_rotational.yaml...");
    return basePath.append("/robots/test_joint_rotational.yaml");
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
