//
// Created by march on 15-5-19.
//

#ifndef MARCH_WS_SRC_HARDWARE_INTERFACE_MARCH_HARDWARE_INTERFACE_INCLUDE_MARCH_HARDWARE_INTERFACE_POWERNETONOFFCOMMAND_H_
#define MARCH_WS_SRC_HARDWARE_INTERFACE_MARCH_HARDWARE_INTERFACE_INCLUDE_MARCH_HARDWARE_INTERFACE_POWERNETONOFFCOMMAND_H_

#include <march_hardware_interface/PowerNetType.h>

class PowerNetOnOffCommand {
  PowerNetType type_;
  bool on_or_off_;
  int net_number_;

public:
  PowerNetOnOffCommand() = default;

  PowerNetOnOffCommand(const PowerNetType type, bool on_or_off, int net_number)
      : type_(type), on_or_off_(on_or_off), net_number_(net_number) {}

  const PowerNetType getType() { return type_; }
  bool isOnOrOff() { return on_or_off_; }
  int getNetNumber() { return net_number_; }
};

#endif // MARCH_WS_SRC_HARDWARE_INTERFACE_MARCH_HARDWARE_INTERFACE_INCLUDE_MARCH_HARDWARE_INTERFACE_POWERNETONOFFCOMMAND_H_
