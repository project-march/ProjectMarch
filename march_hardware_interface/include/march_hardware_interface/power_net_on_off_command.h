// Copyright 2019 Project March
#ifndef MARCH_HARDWARE_INTERFACE_POWERNETONOFFCOMMAND_H
#define MARCH_HARDWARE_INTERFACE_POWERNETONOFFCOMMAND_H

#include <march_hardware_interface/power_net_type.h>

class PowerNetOnOffCommand
{
  PowerNetType type_;
  bool on_or_off_;
  int net_number_;

public:
  PowerNetOnOffCommand()
  {
    reset();
  }

  void reset()
  {
    type_ = PowerNetType();
    on_or_off_ = false;
    net_number_ = -1;
  }

  PowerNetOnOffCommand(const PowerNetType type, bool on_or_off, int net_number)
    : type_(type), on_or_off_(on_or_off), net_number_(net_number)
  {
  }

  const PowerNetType getType() const
  {
    return type_;
  }
  bool isOnOrOff() const
  {
    return on_or_off_;
  }
  int getNetNumber() const
  {
    return net_number_;
  }

  /** @brief Override stream operator for clean printing */
  friend ::std::ostream& operator<<(std::ostream& os, const PowerNetOnOffCommand& powerNetOnOffCommand)
  {
    return os << "PowerNetOnOffCommand(isOnOrOff: " << powerNetOnOffCommand.isOnOrOff()
              << ", netNumber:" << powerNetOnOffCommand.getNetNumber() << ", type: " << powerNetOnOffCommand.getType()
              << ")";
  }
};

#endif  // MARCH_HARDWARE_INTERFACE_POWERNETONOFFCOMMAND_H
