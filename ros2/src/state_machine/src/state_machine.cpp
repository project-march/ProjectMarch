//
// Created by Marco Bak march8 on 1-2-23.
//
#include "state_machine/state_machine.hpp"
using std::placeholders::_1;
using std::placeholders::_2;
using namespace std::chrono_literals;

/**
 * Statemachine creates an instance of the march state machine as used by MVIII.
 *
 * This statemachine consists of all possible state transitions, defined in the m_exo_transitions map.
 * The current state of the exo is stored in m_current_state.
 *
 * @return stateMachine object
 */
StateMachine::StateMachine()
{
    m_current_state = exoState::ForceUnknown;
    // NOTE: Possible improvement is that the states and allowed transitions are loaded from a config file.
    m_exo_transitions = {
        /*{CurrentState, PossibleStates}*/
        { exoState::Sit, { exoState::SitStand, exoState::ForceUnknown, exoState::Error } },
        { exoState::SitStand, { exoState::Sit, exoState::Stand, exoState::ForceUnknown, exoState::Error } },
        { exoState::Stand, { exoState::SitStand, exoState::StandWalk, exoState::ForceUnknown, exoState::Error } },
        { exoState::StandWalk, { exoState::Stand, exoState::Walk, exoState::ForceUnknown, exoState::Error } },
        { exoState::Walk, { exoState::StandWalk, exoState::ForceUnknown, exoState::Error } },
        { exoState::ForceUnknown, { exoState::ForceUnknownStand, exoState::Error } },
        { exoState::ForceUnknownStand, { exoState::Stand, exoState::Error } },

    };
}

/**
 * This function performs a transition between states of the state machine.
 *
 * First the state machine checks if the transition is valid, if this is the case, the transition is made,
 * meaning that the current state is updated accordingly.
 *
 * If there is an invalid transition the exo transitions the the error state, and the safety node is called.
 *
 * @param desired_state
 */
void StateMachine::performTransition(exoState desired_state)
{
    if (isValidTransition(desired_state)) {
        RCLCPP_INFO(rclcpp::get_logger("state_machine"), "Exo state transition succeeded!");
        m_current_state = desired_state;
    } else {
        RCLCPP_ERROR(rclcpp::get_logger("state_machine"), "Invalid State transition!");
        // do ERROR Stuff
        m_current_state = exoState::Error;
    }
}

/**
 * Checks if the transition is allowed by the state-machine.
 *
 * This is checked by iterating through the list of allowed transitions for the current state.
 * During this when the desired state occurs in the list, then it is a valid transition. Otherwise it is invalid.
 *
 * @param desired_state
 * @return  true for valid transition and false for invalid transition.
 */
bool StateMachine::isValidTransition(exoState desired_state)
{
    std::set<exoState> possibleTransitions = m_exo_transitions.at(m_current_state);
    return possibleTransitions.count(desired_state) == 1;
}

/**
 * Gets the current state of the state_machine.
 *
 * @return the current state.
 */
exoState StateMachine::get_current_state()
{
    return m_current_state;
}