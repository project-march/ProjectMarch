#pragma once
#include <string>
#include <map>
#include <vector>
#include <set>
#include "march_mode_machine/exo_mode.hpp"

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
                    { exoMode::LargeWalk, exoMode::SmallWalk, exoMode::BootUp, exoMode::Error, exoMode::VariableStep, exoMode::VariableWalk}},
                { exoMode::LargeWalk, { exoMode::Stand, exoMode::Error} },
                { exoMode::SmallWalk, {exoMode::Stand, exoMode::Error}}, 
                { exoMode::BootUp, { exoMode::Stand } },
                { exoMode::Error, {}},
                { exoMode::VariableStep, { exoMode::Stand, exoMode::Error}}, 
                { exoMode::VariableWalk, { exoMode::Stand, exoMode::Error}}
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