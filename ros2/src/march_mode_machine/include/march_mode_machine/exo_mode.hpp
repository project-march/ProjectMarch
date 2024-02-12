#pragma once
#include <string>
#include <map>
#include <vector>
#include <set>

#define EXO_MODES \
    X(Sit, 0) \
    X(Stand, 1) \
    X(Walk, 2) \
    X(BootUp, 3) \
    X(Error, 4) \
    X(Sideways, 5) \
    X(LargeWalk, 6) \
    X(SmallWalk, 7) \
    X(Ascending, 8) \
    X(Descending, 9) \
    X(VariableWalk, 10) \

#define X(name, value) name = value,
enum class exoMode {
    EXO_MODES
};
#undef X


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
        case exoMode::Ascending: return "Ascending";
        case exoMode::Descending: return "Descending"; 
        case exoMode::VariableWalk: return "VariableWalk"; 
        default: return "Unknown";
    }
}

// Operator overload for output stream
inline std::ostream& operator<<(std::ostream& os, exoMode state) {
    os << toString(state);
    return os;
}

class ExoModeTransitions {
public:
    ExoModeTransitions() {
        transitions = {
            { exoMode::Sit, { exoMode::Stand, exoMode::BootUp, exoMode::Error } },
            { exoMode::Stand, { exoMode::Sit, exoMode::Walk, exoMode::BootUp, exoMode::Error, exoMode::Sideways, exoMode::Ascending, exoMode::Descending} },
            { exoMode::Walk, { exoMode::Stand, exoMode::Error} },
            { exoMode::BootUp, { exoMode::Stand } },
            { exoMode::Error, {}},
            { exoMode::Sideways, { exoMode::Stand}},
            { exoMode::Ascending, { exoMode::Stand}},
            { exoMode::Descending, { exoMode::Stand}}
        };
    }  

    ExoModeTransitions(const std::string& type) {
        if (type == "Joint Angles"){
            transitions = {
                { exoMode::Sit, { exoMode::Stand, exoMode::BootUp, exoMode::Error } },
                { exoMode::Stand, { exoMode::Sit, exoMode::Walk, exoMode::BootUp, exoMode::Error, exoMode::Sideways, exoMode::Ascending, exoMode::Descending} },
                { exoMode::Walk, { exoMode::Stand, exoMode::Error} },
                { exoMode::BootUp, { exoMode::Stand } },
                { exoMode::Error, {}},
                { exoMode::Sideways, { exoMode::Stand}},
                { exoMode::Ascending, { exoMode::Stand}},
                { exoMode::Descending, { exoMode::Stand}}
            };
        }
        else if (type == "Cartesian") {
            transitions = {
                /*{CurrentMode, PossibleModes}*/
                { exoMode::Stand,
                    { exoMode::LargeWalk, exoMode::SmallWalk, exoMode::BootUp, exoMode::Error, exoMode::VariableWalk} },
                { exoMode::LargeWalk, { exoMode::Stand, exoMode::Error} },
                { exoMode::SmallWalk, {exoMode::Stand, exoMode::Error}}, 
                { exoMode::BootUp, { exoMode::Stand } },
                { exoMode::Error, {}},
                { exoMode::VariableWalk, { exoMode::Stand, exoMode::Error}}, 
            };
        }
        else if(type == "Test Joints") {
            transitions = {
                /*{CurrentMode, PossibleModes}*/
                {exoMode::BootUp, {exoMode::Stand}},
                {exoMode::Stand, {exoMode::Walk, exoMode::BootUp}},
                {exoMode::Walk, {exoMode::Stand}}
            };
        }

    }

    std::set<exoMode> getPossibleTransitions(exoMode currentMode) const {
        std::vector<exoMode> vec = transitions.at(currentMode);
        return std::set<exoMode>(vec.begin(), vec.end());
    }

private:
    std::map<exoMode, std::vector<exoMode>> transitions;
};