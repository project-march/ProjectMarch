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
        if(type == "Test Joints") {
            transitions = {
                /*{CurrentMode, PossibleModes}*/
                {exoMode::BootUp, {exoMode::Stand}},
                {exoMode::Stand, {exoMode::Walk, exoMode::BootUp}},
                {exoMode::Walk, {exoMode::Stand}}, 
                { exoMode::Sit, { exoMode::Stand, exoMode::BootUp, exoMode::Error } },
                { exoMode::Stand, { exoMode::Sit, exoMode::Walk, exoMode::BootUp, exoMode::Error, exoMode::Sideways, exoMode::Ascending, exoMode::Descending, exoMode::Hinge} },
                { exoMode::Walk, { exoMode::Stand, exoMode::Error} },
                { exoMode::BootUp, { exoMode::Stand } },
                { exoMode::Error, {}},
                { exoMode::Sideways, { exoMode::Stand}},
                { exoMode::Ascending, { exoMode::Stand}},
                { exoMode::Descending, { exoMode::Stand}},
                { exoMode::Hinge, { exoMode::Stand}}
            };
        }
        else if (type == "Lifecycle nodes"){
            transitions = {
                /*{CurrentMode, PossibleModes}*/
                { exoMode::Stand,
                    { exoMode::LargeWalk, exoMode::SmallWalk, exoMode::BootUp, exoMode::Error, exoMode::VariableStep, exoMode::VariableWalk, exoMode::HighStep1, exoMode::HighStep2, exoMode::HighStep3, exoMode::Ascending, exoMode::Descending, exoMode::Sit, exoMode::Walk, exoMode::Sideways, exoMode::Hinge}},
                { exoMode::LargeWalk, { exoMode::Stand, exoMode::Error} },
                { exoMode::SmallWalk, {exoMode::Stand, exoMode::Error}}, 
                { exoMode::BootUp, { exoMode::Stand } },
                { exoMode::Error, {}},
                { exoMode::VariableStep, { exoMode::Stand, exoMode::Error}}, 
                { exoMode::VariableWalk, { exoMode::Stand, exoMode::Error}}, 
                { exoMode::HighStep1, { exoMode::Stand, exoMode::Error}},
                { exoMode::HighStep2, { exoMode::Stand, exoMode::Error}},
                { exoMode::HighStep3, { exoMode::Stand, exoMode::Error}},
                { exoMode::Ascending, { exoMode::Stand, exoMode::Error}}, 
                { exoMode::Descending, { exoMode::Stand, exoMode::Error}}, 
                { exoMode::Sit, { exoMode::Stand, exoMode::BootUp, exoMode::Error } },
                { exoMode::Walk, { exoMode::Stand, exoMode::Error} },
                { exoMode::Sideways, { exoMode::Stand}}, 
                { exoMode::Hinge, {exoMode::Stand}}
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