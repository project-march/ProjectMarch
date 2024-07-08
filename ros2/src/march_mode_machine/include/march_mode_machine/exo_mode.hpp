#pragma once
#include <string>
#include <vector>

// This is a generated file. Do not edit.

enum class ExoMode {
    Sit = 0,
    Stand = 1,
    Walk = 2,
    BootUp = 3,
    Error = 4,
    SidewaysRight = 5,
    LargeWalk = 6,
    SmallWalk = 7,
    Ascending = 8,
    Descending = 9,
    VariableStep = 10,
    VariableWalk = 11,
    HighStep1 = 12,
    HighStep2 = 13,
    HighStep3 = 14,
    Hinge = 15,
    SidewaysLeft = 16,
};

inline std::string toString(ExoMode state) {
    switch (state) {
        case ExoMode::Sit: return "Sit";
        case ExoMode::Stand: return "Stand";
        case ExoMode::Walk: return "Walk";
        case ExoMode::BootUp: return "BootUp";
        case ExoMode::Error: return "Error";
        case ExoMode::SidewaysRight: return "SidewaysRight";
        case ExoMode::LargeWalk: return "LargeWalk";
        case ExoMode::SmallWalk: return "SmallWalk";
        case ExoMode::Ascending: return "Ascending";
        case ExoMode::Descending: return "Descending";
        case ExoMode::VariableStep: return "VariableStep";
        case ExoMode::VariableWalk: return "VariableWalk";
        case ExoMode::HighStep1: return "HighStep1";
        case ExoMode::HighStep2: return "HighStep2";
        case ExoMode::HighStep3: return "HighStep3";
        case ExoMode::Hinge: return "Hinge";
        case ExoMode::SidewaysLeft: return "SidewaysLeft";
        default: return "Unknown";
    }
}

inline std::ostream& operator<<(std::ostream& os, ExoMode state) {
    os << toString(state);
    return os;
}
