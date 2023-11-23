//
// Created by andrew on 23-11-23.
//

#include "march_input_device/input_device.hpp"
#include <iostream>

IPD::IPD()
    : m_current_state (exoState::ForceUnknown)
{
    std::cout << "IPD succesfully started" << std::endl;
}

exoState IPD::getCurrentState() const
{
    return m_current_state;
}

std::set<exoState> IPD::getAvailableStates() const
{
    return m_available_states;
}

void IPD::askNewState() const
{
    std::cout << "Please enter next state. Available states are: ";

    for (const auto& state : m_available_states) {
        std::cout << state << ", ";
    }

    std::cout << '\n';
}