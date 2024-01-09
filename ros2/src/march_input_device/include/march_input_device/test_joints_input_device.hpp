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
    exoMode getCurrentMode() const;
    std::set<exoMode> getAvailableModes() const;
    void askNewMode() const;
    void setCurrentMode(const exoMode& current_mode);
    void setAvailableModes(const std::set<exoMode>& available_modes);
    void setActuatedJoint(const std::string &actuated_joint);
    std::string getActuatedJoint() const;
    void askNewJoint() const;

private:
    exoMode m_current_mode;
    std::set<exoMode> m_available_modes;
    std::string m_actuated_joint;
    std::vector<std::string> m_available_joints;

};
