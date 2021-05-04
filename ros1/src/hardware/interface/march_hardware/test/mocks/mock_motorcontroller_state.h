#pragma once
#include "march_hardware/motor_controller/motor_controller_state.h"
#include <gmock/gmock.h>
#include <optional>

class MockMotorControllerState : public march::MotorControllerState {
public:
    explicit MockMotorControllerState()
        : MotorControllerState()
    {
    }

    bool isOperational() override
    {
        return true;
    };

    bool hasError() override
    {
        return false;
    }

    std::optional<std::string> getErrorStatus() override
    {
        return "";
    };
    std::string getOperationalState() override
    {
        return "";
    };
};
