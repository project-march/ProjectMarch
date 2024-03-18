#pragma once
#include <string>
#include <vector>

// This is a generated file. Do not edit.

enum class exoMode {
    Sit = 0,
    Stand = 1,
    Walk = 2,
    BootUp = 3,
    Error = 4,
    Sideways = 5,
    LargeWalk = 6,
    SmallWalk = 7,
    Ascending = 8,
    Descending = 9,
    VariableStep = 10,
    VariableWalk = 11,
    HighStep1 = 12,
    HighStep2 = 13,
    HighStep3 = 14,
};

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
        case exoMode::Ascending: return "Ascending";
        case exoMode::Descending: return "Descending";
        case exoMode::VariableStep: return "VariableStep";
        case exoMode::VariableWalk: return "VariableWalk";
        case exoMode::HighStep1: return "HighStep1";
        case exoMode::HighStep2: return "HighStep2";
        case exoMode::HighStep3: return "HighStep3";
        default: return "Unknown";
    }
}

inline std::ostream& operator<<(std::ostream& os, exoMode state) {
    os << toString(state);
    return os;
}
