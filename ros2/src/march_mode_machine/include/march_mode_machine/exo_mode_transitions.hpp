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
            { ExoMode::Sit, { ExoMode::Stand, ExoMode::BootUp, ExoMode::Error } },
            { ExoMode::Stand, { ExoMode::Sit, ExoMode::Walk, ExoMode::BootUp, ExoMode::Error, ExoMode::Sideways, ExoMode::Ascending, ExoMode::Descending} },
            { ExoMode::Walk, { ExoMode::Stand, ExoMode::Error} },
            { ExoMode::BootUp, { ExoMode::Stand } },
            { ExoMode::Error, {}},
            { ExoMode::Sideways, { ExoMode::Stand}},
            { ExoMode::Ascending, { ExoMode::Stand}},
            { ExoMode::Descending, { ExoMode::Stand}}
        };
    }  

    ExoModeTransitions(const std::string& type) {
        if (type == "Joint Angles"){
            transitions = {
                { ExoMode::Sit, { ExoMode::Stand, ExoMode::BootUp, ExoMode::Error } },
                { ExoMode::Stand, { ExoMode::Sit, ExoMode::Walk, ExoMode::BootUp, ExoMode::Error, ExoMode::Sideways, ExoMode::Hinge} },
                { ExoMode::Walk, { ExoMode::Stand, ExoMode::Error} },
                { ExoMode::BootUp, { ExoMode::Stand, ExoMode::Sit } },
                { ExoMode::Error, {}},
                { ExoMode::Sideways, { ExoMode::Stand}},
                { ExoMode::Ascending, { ExoMode::Stand}},
                { ExoMode::Descending, { ExoMode::Stand}},
                { ExoMode::Hinge, { ExoMode::Stand}}
            };
        }
        else if (type == "Cartesian") {
            transitions = {
                /*{CurrentMode, PossibleModes}*/
                { ExoMode::Stand,
                    { ExoMode::LargeWalk, ExoMode::SmallWalk, ExoMode::BootUp, ExoMode::Error, ExoMode::VariableStep, ExoMode::VariableWalk, ExoMode::HighStep1, ExoMode::HighStep2, ExoMode::HighStep3, ExoMode::Ascending, ExoMode::Descending}},
                { ExoMode::LargeWalk, { ExoMode::Stand, ExoMode::Error} },
                { ExoMode::SmallWalk, {ExoMode::Stand, ExoMode::Error}}, 
                { ExoMode::BootUp, { ExoMode::Stand } },
                { ExoMode::Error, {}},
                { ExoMode::VariableStep, { ExoMode::Stand, ExoMode::Error}}, 
                { ExoMode::VariableWalk, { ExoMode::Stand, ExoMode::Error}}, 
                { ExoMode::HighStep1, { ExoMode::Stand, ExoMode::Error}},
                { ExoMode::HighStep2, { ExoMode::Stand, ExoMode::Error}},
                { ExoMode::HighStep3, { ExoMode::Stand, ExoMode::Error}},
                { ExoMode::Ascending, { ExoMode::Stand, ExoMode::Error}}, 
                { ExoMode::Descending, { ExoMode::Stand, ExoMode::Error}}
            };
        }
        else if(type == "Test Joints") {
            transitions = {
                /*{CurrentMode, PossibleModes}*/
                {ExoMode::BootUp, {ExoMode::Stand}},
                {ExoMode::Stand, {ExoMode::Walk, ExoMode::BootUp}},
                {ExoMode::Walk, {ExoMode::Stand}}
            };
        }

    }

    std::set<ExoMode> getPossibleTransitions(ExoMode currentMode) const {
        std::vector<ExoMode> vec = transitions.at(currentMode);
        return std::set<ExoMode>(vec.begin(), vec.end());
    }

private:
    std::map<ExoMode, std::vector<ExoMode>> transitions;
};