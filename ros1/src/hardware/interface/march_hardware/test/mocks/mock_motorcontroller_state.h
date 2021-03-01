#pragma once
#include "march_hardware/motor_controller/motor_controller_state.h"

#include <gmock/gmock.h>

class MockMotorControllerState : public march::MotorControllerState
{
public:
  explicit MockMotorControllerState() : MotorControllerState()
  {
  }

  bool isOk() override
  {
    return true;
  };
  std::string getErrorStatus() override
  {
    return "";
  };
  std::string getOperationalState() override
  {
    return "";
  };

};
