#include "march_mode_machine/mode_machine_cartesian.hpp"

ModeMachineCartesian::ModeMachineCartesian()
{
    RCLCPP_WARN(rclcpp::get_logger("mode_machine_cartesian"), "Mode Machine created");
    m_current_mode = exoMode::BootUp;
    // NOTE: Possible improvement is that the modes and allowed transitions are loaded from a config file.
    m_exo_transitions = ExoModeTransitions("Cartesian");
}

bool ModeMachineCartesian::performTransition(const exoMode& desired_mode)
{
    if (isValidTransition(desired_mode)) {
        m_current_mode = desired_mode;
        return true;
    } else {
        RCLCPP_ERROR(rclcpp::get_logger("mode_machine_cartesian"), "Invalid mode transition!");
        return false;
    }
}

bool ModeMachineCartesian::isValidTransition(const exoMode& desired_mode) const
{
    std::set<exoMode> possibleTransitions =m_exo_transitions.getPossibleTransitions(m_current_mode);
    return possibleTransitions.count(desired_mode) == 1;
}

int ModeMachineCartesian::getCurrentMode() const
{
    return (int)m_current_mode;
}

std::set<exoMode> ModeMachineCartesian::getAvailableModes(exoMode current_mode) const
{
    return m_exo_transitions.getPossibleTransitions(current_mode);
}

void ModeMachineCartesian::setCurrentMode(const exoMode& mode)
{
        m_current_mode = mode;
}
