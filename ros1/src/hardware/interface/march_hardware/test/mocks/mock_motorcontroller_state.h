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

    bool isOperational() const override
    {
        return true;
    };

    bool hasError() const override
    {
        return false;
    }

    std::optional<std::string> getErrorStatus() const override
    {
        return "";
    };
    std::string getOperationalState() const override
    {
        return "";
    };
};
