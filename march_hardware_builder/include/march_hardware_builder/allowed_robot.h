// Copyright 2019 Project March.
#ifndef MARCH_HARDWARE_BUILDER_ALLOWED_ROBOT_H
#define MARCH_HARDWARE_BUILDER_ALLOWED_ROBOT_H
#include <iostream>
#include <string>
#include <ros/package.h>

class AllowedRobot
{
public:
  enum Value : int
  {
    march4,
    march3,
    test_joint_rotational,
    test_joint_linear,
    pdb,
  };

  AllowedRobot() = default;
  explicit AllowedRobot(const std::string& robot_name)
  {
    if (robot_name == "march4")
    {
      this->value = march4;
    }
    else if (robot_name == "march3")
    {
      this->value = march3;
    }
    else if (robot_name == "test_joint_rotational")
    {
      this->value = test_joint_rotational;
    }
    else if (robot_name == "test_joint_linear")
    {
      this->value = test_joint_linear;
    }
    else if (robot_name == "pdb")
    {
      this->value = pdb;
    }
    else
    {
      ROS_WARN_STREAM("Unknown robot " << robot_name);
      this->value = AllowedRobot::test_joint_rotational;
    }
  }

  std::string getFilePath()
  {
    std::string base_path = ros::package::getPath("march_hardware_builder");
    if (this->value == AllowedRobot::march4)
    {
      return base_path.append("/robots/march4.yaml");
    }
    else if (this->value == AllowedRobot::march3)
    {
      return base_path.append("/robots/march3.yaml");
    }
    else if (this->value == AllowedRobot::test_joint_rotational)
    {
      return base_path.append("/robots/test_joint_rotational.yaml");
    }
    else if (this->value == AllowedRobot::test_joint_linear)
    {
      return base_path.append("/robots/test_joint_linear.yaml");
    }
    else if (this->value == AllowedRobot::pdb)
    {
      return base_path.append("/robots/pdb.yaml");
    }
    ROS_ERROR("Robotname not implemented. Using test_joint_rotational.yaml...");
    return base_path.append("/robots/test_joint_rotational.yaml");
  }

  constexpr AllowedRobot(Value allowed_robot) : value(allowed_robot)
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

  friend std::ostream& operator<<(std::ostream& out, const AllowedRobot& c)
  {
    switch (c.value)
    {
      case march4:
        out << "march4";
        break;
      case march3:
        out << "march3";
        break;
      case test_joint_linear:
        out << "test_joint_linear";
        break;
      case test_joint_rotational:
        out << "test_joint_rotational";
        break;
      case pdb:
        out << "pdb";
        break;
      default:
        out << "(Unknown)";
        break;
    }
    return out;
  }

private:
  Value value;
};

#endif  // MARCH_HARDWARE_BUILDER_ALLOWED_ROBOT_H
