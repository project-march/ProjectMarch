//
// Created by Marco Bak, march8 on 1-2-23.
//

#ifndef BUILD_STATE_MACHINE_HPP
#define BUILD_STATE_MACHINE_HPP

#include "march_shared_msgs/msg/gait_type.hpp"
#include "march_shared_msgs/srv/gait_command.hpp"
#include "mujoco_interfaces/msg/ipd_input.hpp"
#include "rclcpp/rclcpp.hpp"
#include <chrono>
#include <cstdio>
#include <march_shared_msgs/msg/error.hpp>
#include <string>

enum class exoState {
    Sit = 0,
    SitStand = 1,
    Stand = 2,
    StandWalk = 3,
    Walk = 4,
    ForceUnknown = 5,
    ForceUnknownStand = 6,
    Error = 7
};

class StateMachine {
public:
    StateMachine();
    void performTransition(exoState desired_state);
    exoState get_current_state();

private:
    bool isValidTransition(exoState desired_state);

    exoState m_current_state;
    std::map<exoState, std::set<exoState>> m_exo_transitions;
};

#endif // BUILD_STATE_MACHINE_HPP