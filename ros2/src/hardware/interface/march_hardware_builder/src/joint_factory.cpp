//
// Created by march on 5-7-22.
//

#include "march_hardware_builder/joint_factory.hpp"

std::vector<march::Joint>
JointFactory::createJoints(const std::vector<std::string> joint_names, const YAML::Node &joints_config) const {
    return std::vector<march::Joint>();
}

std::vector<march::Joint>
JointFactory::createJoints(const std::vector<std::string> joint_names, const YAML::Node &joints_config,
                           const std::vector<march::MotorController> motor_controller) const {
    return std::vector<march::Joint>();
}

std::vector<march::Joint>
JointFactory::createJoints(const std::vector<std::string> joint_names, const YAML::Node &joints_config,
                           const std::vector<march::MotorController> motor_controller,
                           const std::vector<march::TemperatureGES> temprature_ges) const {
    return std::vector<march::Joint>();
}


march::Joint JointFactory::createJoint(const std::string joint_name, const int netnumber,
                                       const march::MotorController motor_controller) const {
    return march::Joint(joint_name, netnumber, motor_controller);
}

march::Joint JointFactory::createJoint(const std::string joint_name, const int netnumber,
                                       const march::MotorController motor_controller,
                                       const march::TemperatureGES temprature_ges) const {
    return march::Joint(__cxx11::basic_string(), 0, std::unique_ptr());
}


