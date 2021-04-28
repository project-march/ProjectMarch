// Copyright 2019 Project March.
#ifndef MARCH_HARDWARE_BUILDER_ALLOWED_ROBOT_H
#define MARCH_HARDWARE_BUILDER_ALLOWED_ROBOT_H
#include <iostream>
#include <string>

#include <ros/package.h>
#include <ros/ros.h>

class AllowedRobot {
public:
    enum Value : int {
        march6,
        march4,
        march3,
        test_joint_rotational,
        test_joint_linear,
        test_joint_rotational_odrive,
        test_joint_linear_odrive,
        pdb,
        pressure_soles
    };

    AllowedRobot() = default;
    // We want non-explicit conversions in this case
    // to increase usability.
    // NOLINTNEXTLINE(hicpp-explicit-conversions)
    AllowedRobot(const std::string& robot_name)
    {
        if (robot_name == "march6") {
            this->value = march6;
        } else if (robot_name == "march4") {
            this->value = march4;
        } else if (robot_name == "march3") {
            this->value = march3;
        } else if (robot_name == "test_joint_rotational") {
            this->value = test_joint_rotational;
        } else if (robot_name == "test_joint_linear") {
            this->value = test_joint_linear;
        } else if (robot_name == "test_joint_rotational_odrive") {
            this->value = test_joint_rotational_odrive;
        } else if (robot_name == "test_joint_linear_odrive") {
            this->value = test_joint_linear_odrive;
        } else if (robot_name == "pdb") {
            this->value = pdb;
        } else if (robot_name == "pressure_soles") {
            this->value = pressure_soles;
        } else {
            ROS_WARN_STREAM("Unknown robot " << robot_name);
            this->value = AllowedRobot::test_joint_rotational;
        }
    }

    std::string getFilePath()
    {
        std::string base_path = ros::package::getPath("march_hardware_builder");
        if (this->value == AllowedRobot::march6) {
            return base_path.append(/*__s=*/"/robots/march6.yaml");
        } else if (this->value == AllowedRobot::march4) {
            return base_path.append(/*__s=*/"/robots/march4.yaml");
        } else if (this->value == AllowedRobot::march3) {
            return base_path.append(/*__s=*/"/robots/march3.yaml");
        } else if (this->value == AllowedRobot::test_joint_rotational) {
            return base_path.append(
                /*__s=*/"/robots/test_joint_rotational.yaml");
        } else if (this->value == AllowedRobot::test_joint_linear) {
            return base_path.append(/*__s=*/"/robots/test_joint_linear.yaml");
        } else if (this->value == AllowedRobot::test_joint_rotational_odrive) {
            return base_path.append(
                /*__s=*/"/robots/test_joint_rotational_odrive.yaml");
        } else if (this->value == AllowedRobot::test_joint_linear_odrive) {
            return base_path.append(
                /*__s=*/"/robots/test_joint_linear_odrive.yaml");
        } else if (this->value == AllowedRobot::pdb) {
            return base_path.append(/*__s=*/"/robots/pdb.yaml");
        } else if (this->value == AllowedRobot::pressure_soles) {
            return base_path.append(/*__s=*/"/robots/pressure_soles.yaml");
        }
        ROS_ERROR(
            "Robotname not implemented. Using test_joint_rotational.yaml...");
        return base_path.append(/*__s=*/"/robots/test_joint_rotational.yaml");
    }

    // We want non-explicit conversions in this case
    // to increase usability.
    // NOLINTNEXTLINE(hicpp-explicit-conversions)
    constexpr AllowedRobot(Value allowed_robot)
        : value(allowed_robot)
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
        switch (c.value) {
            case march6:
                out << "march6";
                break;
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
            case test_joint_linear_odrive:
                out << "test_joint_linear_odrive";
                break;
            case test_joint_rotational_odrive:
                out << "test_joint_rotational_odrive";
                break;
            case pdb:
                out << "pdb";
                break;
            case pressure_soles:
                out << "pressure_soles";
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

#endif // MARCH_HARDWARE_BUILDER_ALLOWED_ROBOT_H