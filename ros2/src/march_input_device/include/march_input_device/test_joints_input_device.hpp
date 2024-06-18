//
// Created by andrew on 23-11-23.
//

#pragma once
#include <set>
#include <string>
#include <vector>
#include "../../march_mode_machine/include/march_mode_machine/exo_mode.hpp"


class TestJointsIPD {
public:
    explicit TestJointsIPD();
    ExoMode getCurrentMode() const;
    std::set<ExoMode> getAvailableModes() const;
    void askNewMode() const;
    void setCurrentMode(const ExoMode& current_mode);
    void setAvailableModes(const std::set<ExoMode>& available_modes);
    void setActuatedJoint(const std::string &actuated_joint);
    std::string getActuatedJoint() const;
    void askNewJoint() const;

private:
    ExoMode m_current_mode;
    std::set<ExoMode> m_available_modes;
    std::string m_actuated_joint;
    std::vector<std::string> m_available_joints;

};
