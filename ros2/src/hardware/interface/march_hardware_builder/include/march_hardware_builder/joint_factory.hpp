//
// Created by march on 5-7-22.
//

#ifndef BUILD_JOINT_FACTORY_HPP
#define BUILD_JOINT_FACTORY_HPP

#include <vector>
#include "hardware_factory.hpp"
#include "march_hardware/joint.h"
#include "yaml-cpp/node/node.h"

class JointFactory : public HardwareFactory {
public:
    std::vector<march::Joint> createJoints(const std::vector<std::string> joint_names,
                                           const YAML::Node &joints_config) const;

    std::vector<march::Joint> createJoints(const std::vector<std::string> joint_names,
                                           const YAML::Node &joints_config,
                                           const std::vector<march::MotorController> motor_controller) const;

    std::vector<march::Joint> createJoints(const std::vector<std::string> joint_names,
                                           const YAML::Node &joints_config,
                                           const std::vector<std::unique_ptr<march::MotorController>> motor_controller,
                                           const std::vector<std::unique_ptr<march::TemperatureGES>> temprature_ges) const;

    march::Joint createJoint(const std::string joint_name,
                             const int netnumber,
                             const march::MotorController motor_controller) const;


    march::Joint createJoint(const std::string joint_name,
                             const int netnumber,
                             const march::MotorController motor_controller,
                             const march::TemperatureGES temprature_ges) const;

private:

};

#endif //BUILD_JOINT_FACTORY_HPP
