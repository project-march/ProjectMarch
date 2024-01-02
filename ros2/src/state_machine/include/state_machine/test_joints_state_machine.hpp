//
// Created by Marco Bak, march8 on 1-2-23.
//

#pragma once

#include "rclcpp/rclcpp.hpp"
#include <chrono>
#include <cstdio>
#include <march_shared_msgs/msg/error.hpp>
#include <string>
#include "state_machine/exo_state.hpp"
#include "state_machine/state_machine.hpp"


class TestJointsStateMachine : public StateMachine {
public:
    explicit TestJointsStateMachine();
};
