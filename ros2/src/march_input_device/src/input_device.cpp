/*Authors: Pleuntje Brons and Andrew Hutani, MIX*/

#include "march_input_device/input_device.hpp"
#include <iostream>

IPD::IPD()
    : m_current_mode (exoMode::BootUp)
{
    std::cout << "IPD succesfully started" << std::endl;
}

exoMode IPD::getCurrentMode() const
{
    return m_current_mode;
}

std::set<exoMode> IPD::getAvailableModes() const
{
    return m_available_modes;
}

void IPD::askNewMode() const
{
    std::cout << "Please enter next mode. Available modes are: ";

    for (const auto& mode : m_available_modes) {
        std::cout << mode << ", ";
    }

    std::cout << '\n';
}

void IPD::setCurrentMode(const exoMode& current_mode)
{
    m_current_mode = current_mode;
}

void IPD::setAvailableModes(const std::set<exoMode>& available_modes)
{
    m_available_modes = available_modes;
}