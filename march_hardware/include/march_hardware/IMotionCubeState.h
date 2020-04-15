// Copyright 2019 Project March.
#ifndef MARCH_HARDWARE_IMOTIONCUBESTATE_H
#define MARCH_HARDWARE_IMOTIONCUBESTATE_H

#include <string>

namespace march
{
class IMCState
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

  IMCState() : value_(UNKNOWN)
  {
  }

  explicit IMCState(uint16_t status) : value_(UNKNOWN)
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
      this->value_ = IMCState::NOT_READY_TO_SWITCH_ON;
    }
    else if (five_bit_masked == switch_on_disabled)
    {
      this->value_ = IMCState::SWITCH_ON_DISABLED;
    }
    else if (six_bit_masked == ready_to_switch_on)
    {
      this->value_ = IMCState::READY_TO_SWITCH_ON;
    }
    else if (six_bit_masked == switched_on)
    {
      this->value_ = IMCState::SWITCHED_ON;
    }
    else if (six_bit_masked == operation_enabled)
    {
      this->value_ = IMCState::OPERATION_ENABLED;
    }
    else if (six_bit_masked == quick_stop_active)
    {
      this->value_ = IMCState::QUICK_STOP_ACTIVE;
    }
    else if (five_bit_masked == fault_reaction_active)
    {
      this->value_ = IMCState::FAULT_REACTION_ACTIVE;
    }
    else if (five_bit_masked == fault)
    {
      this->value_ = IMCState::FAULT;
    }
  }

  std::string getString()
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
    }
  }

  bool operator==(Value v) const
  {
    return this->value_ == v;
  }
  bool operator==(IMCState a) const
  {
    return this->value_ == a.value_;
  }
  bool operator!=(IMCState a) const
  {
    return this->value_ != a.value_;
  }

private:
  Value value_;
};

struct IMotionCubeState
{
public:
  IMotionCubeState() = default;

  std::string statusWord;
  std::string detailedError;
  std::string motionError;
  IMCState state;
  std::string detailedErrorDescription;
  std::string motionErrorDescription;
  float motorCurrent;
  float IMCVoltage;
  int absoluteEncoderValue;
  int incrementalEncoderValue;
  int absoluteVelocity;
  int incrementalVelocity;
};

}  // namespace march

#endif  // MARCH_HARDWARE_IMOTIONCUBESTATE_H
