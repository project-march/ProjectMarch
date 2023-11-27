//
// Created by andrew on 23-11-23.
//

#pragma once
#include <set>
#include <string>
#include "../../state_machine/include/state_machine/exo_state.hpp"


class IPD {
public:
    explicit IPD();
    exoState getCurrentState() const;
    std::set<exoState> getAvailableStates() const;
    void askNewState() const;
    void setCurrentState(const exoState& current_state);
    void setAvailableStates(const std::set<exoState>& available_states);

private:
    exoState m_current_state;
    std::set<exoState> m_available_states;

};
