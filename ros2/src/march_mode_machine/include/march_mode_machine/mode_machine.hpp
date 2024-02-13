//
// Created by Marco Bak, march8 on 1-2-23.
//

#pragma once

#include "march_shared_msgs/srv/gait_command.hpp"
#include "rclcpp/rclcpp.hpp"
#include <chrono>
#include <cstdio>
#include <march_shared_msgs/msg/error.hpp>
#include <string>
#include "march_mode_machine/exo_mode_transitions.hpp"


class ModeMachine {
public:
    explicit ModeMachine();
    bool performTransition(const exoMode& desired_mode);
    bool isValidTransition(const exoMode& desired_mode) const;
    int getCurrentMode() const;
    std::set<exoMode> getAvailableModes(exoMode current_mode) const;
    void setCurrentMode(const exoMode& mode);

protected:
    ExoModeTransitions m_exo_transitions;

private:
    exoMode m_current_mode;

};
