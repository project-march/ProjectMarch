// Copyright 2019 Project March.
#ifndef MARCH_HARDWARE_HIGH_VOLTAGE_H
#define MARCH_HARDWARE_HIGH_VOLTAGE_H
#include "march_hardware/communication/ethercat/pdo_interface.h"
#include "net_driver_offsets.h"
#include "net_monitor_offsets.h"

#include <cstdint>
#include <iostream>

namespace march
{
class HighVoltage
{
private:
  PdoSlaveInterface& pdo_;
  NetMonitorOffsets netMonitoringOffsets;
  NetDriverOffsets netDriverOffsets;

  uint8_t getNetsOperational();

public:
  HighVoltage(PdoSlaveInterface& pdo, NetMonitorOffsets netMonitoringOffsets, NetDriverOffsets netDriverOffsets);

  float getNetCurrent();
  bool getNetOperational(int netNumber);
  bool getOvercurrentTrigger(int netNumber);
  bool getHighVoltageEnabled();
  void setNetOnOff(bool on, int netNumber);
  void enableDisableHighVoltage(bool enable);

  /** @brief Override comparison operator */
  friend bool operator==(const HighVoltage& lhs, const HighVoltage& rhs)
  {
    return lhs.netDriverOffsets == rhs.netDriverOffsets && lhs.netMonitoringOffsets == rhs.netMonitoringOffsets;
  }

  /** @brief Override stream operator for clean printing */
  friend ::std::ostream& operator<<(std::ostream& os, const HighVoltage& highVoltage)
  {
    return os << "HighVoltage(netMonitoringOffsets: " << highVoltage.netMonitoringOffsets << ", "
              << "netDriverOffsets: " << highVoltage.netDriverOffsets << ")";
  }
};

}  // namespace march
#endif  // MARCH_HARDWARE_HIGH_VOLTAGE_H
