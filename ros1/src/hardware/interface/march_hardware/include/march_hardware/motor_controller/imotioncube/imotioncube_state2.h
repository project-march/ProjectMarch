// Copyright 2019 Project March.
#ifndef MARCH_HARDWARE_IMOTIONCUBE_STATE_H
#define MARCH_HARDWARE_IMOTIONCUBE_STATE_H

#include <string>
#include "march_hardware/motor_controller/motor_controller_state.h"

namespace march
{
class IMotionCubeState : public MotorControllerState
{
public:
  IMotionCubeState() = default;
  IMotionCubeState(float motor_current, float motor_voltage,
                   double absolute_angle_iu, double incremental_angle_iu,
                   double absolute_velocity_iu, double incremental_velocity_iu,
                   double absolute_angle_rad, double incremental_angle_rad,
                   double absolute_velocity_rad, double incremental_velocity_rad,
                   uint16_t status_word, std::string motion_error, std::string detailed_error,
                   std::string second_detailed_error, std::string detailed_error_description,
                   std::string motion_error_description, std::string second_detailed_error_description)
    : MotorControllerState(motor_current, motor_voltage, absolute_angle_iu, incremental_angle_iu,
                             absolute_velocity_iu, incremental_velocity_iu, absolute_angle_rad, incremental_angle_rad,
                             absolute_velocity_rad, incremental_velocity_rad, IMCStateOfOperation(status_word))
    , motion_error_(motion_error)
    , detailed_error_(detailed_error)
    , second_detailed_error_(second_detailed_error)
    , detailed_error_description_(detailed_error_description)
    , motion_error_description_(motion_error_description)
    , second_detailed_error_description_(second_detailed_error_description)
  {}

  bool checkState() override
  {
    return !(this->state_of_operation_ == march::IMCStateOfOperation::FAULT);
  }

  std::string getErrorStatus() override
  {
    std::ostringstream error_stream;
    std::string state = this->state_of_operation_.getString().c_str();

    error_stream << "State: " << state << "\nMotion Error: " << this->motion_error_description_ << " ("
                 << this->motion_error_ << ")\nDetailed Error: " << this->detailed_error_description_ << " ("
                 << this->detailed_error_ << ")\nSecond Detailed Error: " << this->second_detailed_error_description_ << " ("
                 << this->second_detailed_error_ << ")";
    return error_stream.str();
  }

  std::string motion_error_;
  std::string detailed_error_;
  std::string second_detailed_error_;
  std::string detailed_error_description_;
  std::string motion_error_description_;
  std::string second_detailed_error_description_;
};
class IMCStateOfOperation : public MotorControllerStateOfOperation
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

  std::string getString() override
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
    return this->value_ == v;
  }
  bool operator==(IMCStateOfOperation a) const
  {
    return this->value_ == a.value_;
  }
  bool operator!=(IMCStateOfOperation a) const
  {
    return this->value_ != a.value_;
  }

private:
  Value value_;
};
}  // namespace march

#endif  // MARCH_HARDWARE_IMOTIONCUBE_STATE_OF_OPERATION_H
