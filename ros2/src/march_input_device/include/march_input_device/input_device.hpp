//
// Created by andrew on 23-11-23.
//

#pragma once
#include <set>
#include <string>


//TODO: Move this class to its own file structure in state machine
enum class exoState { Sit = 0, Stand = 1, Walk = 2, StepClose = 3, ForceUnknown = 4, Error = 5 };

// Function to convert exoState enum to string
inline std::string toString(exoState state) {
    switch (state) {
        case exoState::Sit: return "Sit";
        case exoState::Stand: return "Stand";
        case exoState::Walk: return "Walk";
        case exoState::StepClose: return "StepClose";
        case exoState::ForceUnknown: return "ForceUnknown";
        case exoState::Error: return "Error";
        default: return "Unknown State";
    }
}

// Overload operator<< for exoState
inline std::ostream& operator<<(std::ostream& os, exoState state) {
    os << toString(state);
    return os;
}

class IPD {
public:
    explicit IPD();
    exoState getCurrentState() const;
    std::set<exoState> getAvailableStates() const;
    void askNewState() const;
    void setCurrentState(const exoState& current_state);
    void setAvailableStates(const std::set<exoState>& available_states);

private:
    exoState m_current_state;
    std::set<exoState> m_available_states;

};
