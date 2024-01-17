//
// Created by andrew on 23-11-23.
//

#pragma once
#include <set>
#include <string>
#include "../../march_mode_machine/include/march_mode_machine/exo_mode.hpp"


class IPD {
public:
    explicit IPD();
    exoMode getCurrentMode() const;
    std::set<exoMode> getAvailableModes() const;
    void askNewMode() const;
    void setCurrentMode(const exoMode& current_mode);
    void setAvailableModes(const std::set<exoMode>& available_modes);

private:
    exoMode m_current_mode;
    std::set<exoMode> m_available_modes;

};
