//
// Created by Marco Bak march8 on 1-2-23.
//
#include "state_machine/state_machine.hpp"
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
    RCLCPP_WARN(rclcpp::get_logger("state_machine"), "State Machine created");
    m_current_state = exoState::BootUp;
    // NOTE: Possible improvement is that the states and allowed transitions are loaded from a config file.
    m_exo_transitions = {
        /*{CurrentState, PossibleStates}*/
        { exoState::Sit, { exoState::Stand, exoState::Error } },
        { exoState::Stand,
            { exoState::Sit, exoState::Walk, exoState::BootUp, exoState::Error, exoState::Sideways} },
        { exoState::Walk, { exoState::Stand, exoState::Error} },
        { exoState::BootUp, { exoState::Stand} },
        { exoState::Error, {}}, 
        { exoState::Sideways, { exoState::Stand, exoState::Error}}

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
bool StateMachine::performTransition(const exoState& desired_state)
{
    if (isValidTransition(desired_state)) {
        m_current_state = desired_state;
        return true;
    } else {
        RCLCPP_ERROR(rclcpp::get_logger("state_machine"), "Invalid State transition!");
        return false;
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
bool StateMachine::isValidTransition(const exoState& desired_state) const
{
    std::set<exoState> possibleTransitions = m_exo_transitions.at(m_current_state);
    return possibleTransitions.count(desired_state) == 1;
}

/**
 * Gets the current state of the state_machine.
 *
 * @return the current state.
 */
int StateMachine::getCurrentState() const
{
    return (int)m_current_state;
}

std::set<exoState> StateMachine::getAvailableStates(exoState currentState) const
{
    return m_exo_transitions.at(currentState);
}

void StateMachine::setCurrentState(const exoState& state)
{
        m_current_state = state;
}

void StateMachine::setExoTransitions(const std::map<exoState, std::set<exoState>>& transitions)
{
        m_exo_transitions = transitions;
}