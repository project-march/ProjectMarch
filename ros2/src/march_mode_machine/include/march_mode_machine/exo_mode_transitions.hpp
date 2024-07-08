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
            { ExoMode::Stand, { ExoMode::Sit, ExoMode::Walk, ExoMode::BootUp, ExoMode::Error, ExoMode::SidewaysRight, ExoMode::Ascending, ExoMode::Descending} },
            { ExoMode::Walk, { ExoMode::Stand, ExoMode::Error} },
            { ExoMode::BootUp, { ExoMode::Stand } },
            { ExoMode::Error, {}},
            { ExoMode::SidewaysRight, { ExoMode::Stand}},
            { ExoMode::Ascending, { ExoMode::Stand}},
            { ExoMode::Descending, { ExoMode::Stand}}
        };
    }  

    ExoModeTransitions(const std::string& type) {
        if(type == "Test Joints") {
            transitions = {
                /*{CurrentMode, PossibleModes}*/
                {ExoMode::BootUp, {ExoMode::Stand}},
                {ExoMode::Stand, {ExoMode::Walk, ExoMode::BootUp}},
                {ExoMode::Walk, {ExoMode::Stand}}, 
                { ExoMode::Sit, { ExoMode::Stand, ExoMode::BootUp, ExoMode::Error } },
                { ExoMode::Stand, { ExoMode::Sit, ExoMode::Walk, ExoMode::BootUp, ExoMode::Error, ExoMode::SidewaysRight, ExoMode::Ascending, ExoMode::Descending, ExoMode::Hinge} },
                { ExoMode::Walk, { ExoMode::Stand, ExoMode::Error} },
                { ExoMode::BootUp, { ExoMode::Stand } },
                { ExoMode::Error, {}},
                { ExoMode::SidewaysRight, { ExoMode::Stand}},
                { ExoMode::Ascending, { ExoMode::Stand}},
                { ExoMode::Descending, { ExoMode::Stand}},
                { ExoMode::Hinge, { ExoMode::Stand}}
            };
        }
        else if (type == "Lifecycle nodes"){
            transitions = {
                /*{CurrentMode, PossibleModes}*/
                { ExoMode::Stand,
                    { ExoMode::LargeWalk, ExoMode::SmallWalk, ExoMode::BootUp, ExoMode::Error, ExoMode::VariableStep, ExoMode::VariableWalk, ExoMode::HighStep1, ExoMode::HighStep2, ExoMode::HighStep3, ExoMode::Ascending, ExoMode::Descending, ExoMode::Sit, ExoMode::SidewaysLeft, ExoMode::SidewaysRight, ExoMode::Hinge}},
                { ExoMode::LargeWalk, { ExoMode::Stand, ExoMode::Error} },
                { ExoMode::SmallWalk, {ExoMode::Stand, ExoMode::Error}}, 
                { ExoMode::BootUp, { ExoMode::Stand, ExoMode::Sit} },
                { ExoMode::Error, {}},
                { ExoMode::VariableStep, { ExoMode::Stand, ExoMode::Error}}, 
                { ExoMode::VariableWalk, { ExoMode::Stand, ExoMode::Error}}, 
                { ExoMode::HighStep1, { ExoMode::Stand, ExoMode::Error}},
                { ExoMode::HighStep2, { ExoMode::Stand, ExoMode::Error}},
                { ExoMode::HighStep3, { ExoMode::Stand, ExoMode::Error}},
                { ExoMode::Ascending, { ExoMode::Stand, ExoMode::Error}}, 
                { ExoMode::Descending, { ExoMode::Stand, ExoMode::Error}}, 
                { ExoMode::Sit, { ExoMode::Stand, ExoMode::BootUp, ExoMode::Error } },
                { ExoMode::Walk, { ExoMode::Stand, ExoMode::Error} },
                { ExoMode::SidewaysLeft, { ExoMode::Stand}}, 
                { ExoMode::SidewaysRight, { ExoMode::Stand}},
                { ExoMode::Hinge, {ExoMode::Stand}}
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