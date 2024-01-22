#pragma once
#include <string>

enum class exoMode { Sit = 0, Stand = 1, Walk = 2, BootUp = 3, Error = 4, Sideways = 5, LargeWalk = 6, SmallWalk = 7};

// toString function
inline std::string toString(exoMode state) {
    switch (state) {
        case exoMode::Sit: return "Sit";
        case exoMode::Stand: return "Stand";
        case exoMode::Walk: return "Walk";
        case exoMode::BootUp: return "BootUp";
        case exoMode::Error: return "Error";
        case exoMode::Sideways: return "Sideways";
        case exoMode::LargeWalk: return "LargeWalk"; 
        case exoMode::SmallWalk: return "SmallWalk"; 
        default: return "Unknown";
    }
}

// Operator overload for output stream
inline std::ostream& operator<<(std::ostream& os, exoMode state) {
    os << toString(state);
    return os;
}