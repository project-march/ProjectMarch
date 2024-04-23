/*Authors: Andrew Hutani, MIX

This class is a seperate IPD for the testing of single joints. It has an additional member variable to store the actuated joint.

This class is only used in the test_joints launch file.

*/

#include "march_input_device/test_joints_input_device.hpp"
#include <iostream>

TestJointsIPD::TestJointsIPD()
    : m_current_mode (exoMode::BootUp)
{
    std::cout << "IPD succesfully started" << std::endl;
}

exoMode TestJointsIPD::getCurrentMode() const
{
    return m_current_mode;
}

std::set<exoMode> TestJointsIPD::getAvailableModes() const
{
    return m_available_modes;
}

void TestJointsIPD::askNewMode() const
{
    std::cout << "Please enter next mode. Available modes are: ";

    for (const auto& mode : m_available_modes) {
        std::cout << mode << ", ";
    }

    std::cout << '\n';
}

void TestJointsIPD::setCurrentMode(const exoMode& current_mode)
{
    m_current_mode = current_mode;
}

void TestJointsIPD::setAvailableModes(const std::set<exoMode>& available_modes)
{
    m_available_modes = available_modes;
}

void TestJointsIPD::setActuatedJoint(const std::string &actuated_joint)
{
    m_actuated_joint = actuated_joint;
}

std::string TestJointsIPD::getActuatedJoint() const
{
    return m_actuated_joint;
}

void TestJointsIPD::askNewJoint() const
{
    std::cout << "Please enter next joint. Available joints are: ";

    for (const auto& joint : m_available_joints) {
        std::cout << joint << ", ";
    }

    std::cout << '\n';
}