//
// Created by Marco Bak march8 on 1-2-23.
//
#include "march_mode_machine/mode_machine.hpp"
using namespace std::chrono_literals;

/**
 * ModeMachine creates an instance of the march mode machine as used by MVIII.
 *
 * This ModeMachine consists of all possible mode transitions, defined in the m_exo_transitions map.
 * The current mode of the exo is stored in m_current_mode.
 *
 * @return ModeMachine object
 */
ModeMachine::ModeMachine()
{
    RCLCPP_WARN(rclcpp::get_logger("mode_machine"), "Mode Machine created");
    m_current_mode = exoMode::BootUp;
    // NOTE: Possible improvement is that the modes and allowed transitions are loaded from a config file.
    m_exo_transitions = {
        /*{CurrentMode, PossibleModes}*/
        { exoMode::Sit, { exoMode::Stand, exoMode::BootUp, exoMode::Error } },
        { exoMode::Stand,
            { exoMode::Sit, exoMode::Walk, exoMode::BootUp, exoMode::Error, exoMode::Sideways, exoMode::Ascending, exoMode::Descending} },
        { exoMode::Walk, { exoMode::Stand, exoMode::Error} },
        { exoMode::BootUp, { exoMode::Stand } },
        { exoMode::Error, {}},
        { exoMode::Sideways, { exoMode::Stand}},
        { exoMode::Ascending, { exoMode::Stand}},
        { exoMode::Descending, { exoMode::Stand}}

    };
    
}

/**
 * This function performs a transition between modes of the mode machine.
 *
 * First the mode machine checks if the transition is valid, if this is the case, the transition is made,
 * meaning that the current mode is updated accordingly.
 *
 * If there is an invalid transition the exo transitions the the error mode, and the safety node is called.
 *
 * @param desired_mode
 */
bool ModeMachine::performTransition(const exoMode& desired_mode)
{
    if (isValidTransition(desired_mode)) {
        m_current_mode = desired_mode;
        return true;
    } else {
        RCLCPP_ERROR(rclcpp::get_logger("mode_machine"), "Invalid mode transition!");
        return false;
    }
}

/**
 * Checks if the transition is allowed by the mode-machine.
 *
 * This is checked by iterating through the list of allowed transitions for the current mode.
 * During this when the desired mode occurs in the list, then it is a valid transition. Otherwise it is invalid.
 *
 * @param desired_mode
 * @return  true for valid transition and false for invalid transition.
 */
bool ModeMachine::isValidTransition(const exoMode& desired_mode) const
{
    std::set<exoMode> possibleTransitions = m_exo_transitions.at(m_current_mode);
    return possibleTransitions.count(desired_mode) == 1;
}

/**
 * Gets the current mode of the mode_machine.
 *
 * @return the current mode.
 */
int ModeMachine::getCurrentMode() const
{
    return (int)m_current_mode;
}

std::set<exoMode> ModeMachine::getAvailableModes(exoMode current_mode) const
{
    return m_exo_transitions.at(current_mode);
}

void ModeMachine::setCurrentMode(const exoMode& mode)
{
        m_current_mode = mode;
}

void ModeMachine::setExoTransitions(const std::map<exoMode, std::set<exoMode>>& transitions)
{
        m_exo_transitions = transitions;
}