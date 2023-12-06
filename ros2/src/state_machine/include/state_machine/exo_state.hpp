#pragma once
#include <string>

enum class exoState { Sit = 0, Stand = 1, Walk = 2, StepClose = 3, BootUp = 4, Error = 5 };

// toString function
inline std::string toString(exoState state) {
    switch (state) {
        case exoState::Sit: return "Sit";
        case exoState::Stand: return "Stand";
        case exoState::Walk: return "Walk";
        case exoState::StepClose: return "StepClose";
        case exoState::BootUp: return "BootUp";
        case exoState::Error: return "Error";
        default: return "Unknown";
    }
}

// Operator overload for output stream
inline std::ostream& operator<<(std::ostream& os, exoState state) {
    os << toString(state);
    return os;
}