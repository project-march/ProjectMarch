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
    test_setup,
    pdb_test_setup,
    pdb_and_joint_test_setup
  };

  AllowedRobot() = default;
  explicit AllowedRobot(const std::string& robotName)
  {
    if (robotName == "march3")
    {
      this->value = march3;
    }
    else if (robotName == "test_setup")
    {
      this->value = test_setup;
    }
    else if (robotName == "pdb_test_setup")
    {
      this->value = pdb_test_setup;
    }
    else
    {
      ROS_ASSERT_MSG(false, "Unknown robot %s", robotName.c_str());
      this->value = AllowedRobot::test_setup;
    }
  }

  std::string getFilePath()
  {
    std::string basePath = ros::package::getPath("march_hardware_builder");
    if (this->value == AllowedRobot::test_setup)
    {
      return basePath.append("/src/robots/test_setup.yaml");
    }
    else if (this->value == AllowedRobot::march3)
    {
        return basePath.append("/src/robots/march3.yaml");
    }
    else if (this->value == AllowedRobot::pdb_test_setup)
    {
      return basePath.append("/src/robots/power_distribution_board_setup.yaml");
    }
    else if (this->value == AllowedRobot::pdb_and_joint_test_setup)
    {
      return basePath.append("/src/robots/power_distribution_board_and_joint_setup.yaml");
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
