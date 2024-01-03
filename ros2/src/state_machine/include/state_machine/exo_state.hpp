#pragma once
#include <string>

enum class exoState { Sit = 0, Stand = 1, Walk = 2, BootUp = 3, Error = 4, Sideways = 5};

// toString function
inline std::string toString(exoState state) {
    switch (state) {
        case exoState::Sit: return "Sit";
        case exoState::Stand: return "Stand";
        case exoState::Walk: return "Walk";
        case exoState::BootUp: return "BootUp";
        case exoState::Error: return "Error";
        case exoState::Sideways: return "Sideways"; 
        default: return "Unknown";
    }
}

// Operator overload for output stream
inline std::ostream& operator<<(std::ostream& os, exoState state) {
    os << toString(state);
    return os;
}