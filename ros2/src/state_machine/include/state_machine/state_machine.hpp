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

enum class exoState { Sit = 0, Stand = 1, Walk = 2, StepClose = 3, ForceUnknown = 4, Error = 5 };

class StateMachine {
public:
    explicit StateMachine();
    bool performTransition(const exoState& desired_state);
    bool isValidTransition(const exoState& desired_state) const;
    int getCurrentState() const;

private:
    exoState m_current_state;
    std::map<exoState, std::set<exoState>> m_exo_transitions;
};
