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
    ExoMode getCurrentMode() const;
    std::set<ExoMode> getAvailableModes() const;
    void askNewMode() const;
    void setCurrentMode(const ExoMode& current_mode);
    void setAvailableModes(const std::set<ExoMode>& available_modes);

private:
    ExoMode m_current_mode;
    std::set<ExoMode> m_available_modes;

};
