/*Authors: Andrew Hutani, MIX

This class is a seperate IPD for the testing of single joints. It has an additional member variable to store the actuated joint.

This class is only used in the test_joints launch file.

*/

#include "march_input_device/test_joints_input_device.hpp"
#include <iostream>

TestJointsIPD::TestJointsIPD()
    : m_current_state (exoState::BootUp)
{
    std::cout << "IPD succesfully started" << std::endl;
}

exoState TestJointsIPD::getCurrentState() const
{
    return m_current_state;
}

std::set<exoState> TestJointsIPD::getAvailableStates() const
{
    return m_available_states;
}

void TestJointsIPD::askNewState() const
{
    std::cout << "Please enter next state. Available states are: ";

    for (const auto& state : m_available_states) {
        std::cout << state << ", ";
    }

    std::cout << '\n';
}

void TestJointsIPD::setCurrentState(const exoState& current_state)
{
    m_current_state = current_state;
}

void TestJointsIPD::setAvailableStates(const std::set<exoState>& available_states)
{
    m_available_states = available_states;
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