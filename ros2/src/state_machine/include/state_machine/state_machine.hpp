//
// Created by Marco Bak, march8 on 1-2-23.
//

#ifndef BUILD_STATE_MACHINE_HPP
#define BUILD_STATE_MACHINE_HPP

#include "march_shared_msgs/srv/gait_command.hpp"
#include "rclcpp/rclcpp.hpp"
#include <chrono>
#include <cstdio>
#include <march_shared_msgs/msg/error.hpp>
#include <string>

enum class exoState { Sit = 0, Stand = 1, Walk = 2, StepClose = 3, ForceUnknown = 4, Error = 5 };

class StateMachine {
public:
    StateMachine();
    bool performTransition(exoState desired_state);
    int get_current_state();

private:
    bool isValidTransition(exoState desired_state);

    exoState m_current_state;
    std::map<exoState, std::set<exoState>> m_exo_transitions;
};

#endif // BUILD_STATE_MACHINE_HPP
