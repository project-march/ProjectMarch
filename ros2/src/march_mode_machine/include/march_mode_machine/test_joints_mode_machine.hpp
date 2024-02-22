//
// Created by Marco Bak, march8 on 1-2-23.
//

#pragma once

#include "rclcpp/rclcpp.hpp"
#include <chrono>
#include <cstdio>
#include <march_shared_msgs/msg/error.hpp>
#include <string>
#include "march_mode_machine/exo_mode_transitions.hpp"
#include "march_mode_machine/mode_machine.hpp"


class TestJointsModeMachine : public ModeMachine {
public:
    explicit TestJointsModeMachine();
};
