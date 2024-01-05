//
// Created by andrew on 23-11-23.
//

#pragma once
#include <set>
#include <string>
#include <vector>
#include "../../state_machine/include/state_machine/exo_state.hpp"


class TestJointsIPD {
public:
    explicit TestJointsIPD();
    exoState getCurrentState() const;
    std::set<exoState> getAvailableStates() const;
    void askNewState() const;
    void setCurrentState(const exoState& current_state);
    void setAvailableStates(const std::set<exoState>& available_states);
    void setActuatedJoint(const std::string &actuated_joint);
    std::string getActuatedJoint() const;
    void askNewJoint() const;

private:
    exoState m_current_state;
    std::set<exoState> m_available_states;
    std::string m_actuated_joint;
    std::vector<std::string> m_available_joints;

};
