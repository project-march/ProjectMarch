// Copyright 2019 Project March.
#ifndef MARCH_HARDWARE_IMOTIONCUBE_STATE_H
#define MARCH_HARDWARE_IMOTIONCUBE_STATE_H

#include <bitset>
#include <string>
#include <optional>
#include "march_hardware/motor_controller/motor_controller_state.h"
#include "march_hardware/error/hardware_exception.h"
#include "march_hardware/error/motor_controller_error.h"

namespace march
{
class IMCStateOfOperation
{
public:
  enum Value
  {
    NOT_READY_TO_SWITCH_ON,
    SWITCH_ON_DISABLED,
    READY_TO_SWITCH_ON,
    SWITCHED_ON,
    OPERATION_ENABLED,
    QUICK_STOP_ACTIVE,
    FAULT_REACTION_ACTIVE,
    FAULT,
    UNKNOWN,
  };

  IMCStateOfOperation() : value_(UNKNOWN)
  {
  }

  explicit IMCStateOfOperation(uint16_t status) : value_(UNKNOWN)
  {
    const uint16_t five_bit_mask = 0b0000000001001111;
    const uint16_t six_bit_mask = 0b0000000001101111;

    const uint16_t not_ready_switch_on = 0b0000000000000000;
    const uint16_t switch_on_disabled = 0b0000000001000000;
    const uint16_t ready_to_switch_on = 0b0000000000100001;
    const uint16_t switched_on = 0b0000000000100011;
    const uint16_t operation_enabled = 0b0000000000100111;
    const uint16_t quick_stop_active = 0b0000000000000111;
    const uint16_t fault_reaction_active = 0b0000000000001111;
    const uint16_t fault = 0b0000000000001000;

    const uint16_t five_bit_masked = (status & five_bit_mask);
    const uint16_t six_bit_masked = (status & six_bit_mask);

    if (five_bit_masked == not_ready_switch_on)
    {
      this->value_ = IMCStateOfOperation::NOT_READY_TO_SWITCH_ON;
    }
    else if (five_bit_masked == switch_on_disabled)
    {
      this->value_ = IMCStateOfOperation::SWITCH_ON_DISABLED;
    }
    else if (six_bit_masked == ready_to_switch_on)
    {
      this->value_ = IMCStateOfOperation::READY_TO_SWITCH_ON;
    }
    else if (six_bit_masked == switched_on)
    {
      this->value_ = IMCStateOfOperation::SWITCHED_ON;
    }
    else if (six_bit_masked == operation_enabled)
    {
      this->value_ = IMCStateOfOperation::OPERATION_ENABLED;
    }
    else if (six_bit_masked == quick_stop_active)
    {
      this->value_ = IMCStateOfOperation::QUICK_STOP_ACTIVE;
    }
    else if (five_bit_masked == fault_reaction_active)
    {
      this->value_ = IMCStateOfOperation::FAULT_REACTION_ACTIVE;
    }
    else if (five_bit_masked == fault)
    {
      this->value_ = IMCStateOfOperation::FAULT;
    }
  }

  std::string toString()
  {
    switch (this->value_)
    {
      case NOT_READY_TO_SWITCH_ON:
        return "Not Ready To Switch On";
      case SWITCH_ON_DISABLED:
        return "Switch On Disabled";
      case READY_TO_SWITCH_ON:
        return "Ready to Switch On";
      case SWITCHED_ON:
        return "Switched On";
      case OPERATION_ENABLED:
        return "Operation Enabled";
      case QUICK_STOP_ACTIVE:
        return "Quick Stop Active";
      case FAULT_REACTION_ACTIVE:
        return "Fault Reaction Active";
      case FAULT:
        return "Fault";
      case UNKNOWN:
        return "Not in a recognized IMC state";
      default:
        return "Not in a recognized IMC state";
    }
  }

  bool operator==(Value v) const
  {
    return value_ == v;
  }
  bool operator==(IMCStateOfOperation a) const
  {
    return value_ == a.value_;
  }
  bool operator!=(IMCStateOfOperation a) const
  {
    return value_ != a.value_;
  }

  Value value_;
};
class IMotionCubeState : public MotorControllerState
{
public:
  IMotionCubeState() = default;

  bool isOperational() override
  {
    return state_of_operation_.value_ != march::IMCStateOfOperation::FAULT;
  }

  bool hasError() override
  {
    return motion_error_ || detailed_error_ || second_detailed_error_;
  }

  std::optional<std::string> getErrorStatus() override
  {
    if (hasError())
    {
      std::ostringstream error_stream;
      error_stream << "\nMotion Error: " <<  error::parseError(motion_error_, error::ErrorRegister::IMOTIONCUBE_MOTION_ERROR)
                   << " (" << std::bitset<16>(motion_error_).to_string() << ")"
                   << "\nDetailed Error: " << error::parseError(detailed_error_, error::ErrorRegister::IMOTIONCUBE_DETAILED_MOTION_ERROR)
                   << " (" << std::bitset<16>(motion_error_).to_string() << ")"
                   << "\nSecond Detailed Error: " << error::parseError(second_detailed_error_, error::ErrorRegister::IMOTIONCUBE_SECOND_DETAILED_MOTION_ERROR)
                   << " (" << std::bitset<16>(motion_error_).to_string() << ")";
      return error_stream.str();
    }
    else
    {
      return std::nullopt;
    }
  }

  std::string getOperationalState() override
  {
    return state_of_operation_.toString();
  }

  IMCStateOfOperation state_of_operation_;
  uint16_t motion_error_;
  uint16_t detailed_error_;
  uint16_t second_detailed_error_;
};
}  // namespace march

#endif  // MARCH_HARDWARE_IMOTIONCUBE_STATE_OF_OPERATION_H
