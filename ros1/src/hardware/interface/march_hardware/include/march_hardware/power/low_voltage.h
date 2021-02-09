// Copyright 2019 Project March.
#ifndef MARCH_HARDWARE_LOW_VOLTAGE_H
#define MARCH_HARDWARE_LOW_VOLTAGE_H
#include "march_hardware/communication/ethercat/pdo_interface.h"
#include "net_driver_offsets.h"
#include "net_monitor_offsets.h"

#include <cstdint>
#include <iostream>

namespace march
{
class LowVoltage
{
private:
  PdoSlaveInterface& pdo_;
  NetMonitorOffsets netMonitoringOffsets;
  NetDriverOffsets netDriverOffsets;

  uint8_t getNetsOperational();

public:
  LowVoltage(PdoSlaveInterface& pdo, NetMonitorOffsets netMonitoringOffsets, NetDriverOffsets netDriverOffsets);

  float getNetCurrent(int netNumber);
  bool getNetOperational(int netNumber);
  void setNetOnOff(bool on, int netNumber);

  /** @brief Override comparison operator */
  friend bool operator==(const LowVoltage& lhs, const LowVoltage& rhs)
  {
    return lhs.netDriverOffsets == rhs.netDriverOffsets && lhs.netMonitoringOffsets == rhs.netMonitoringOffsets;
  }

  /** @brief Override stream operator for clean printing */
  friend ::std::ostream& operator<<(std::ostream& os, const LowVoltage& lowVoltage)
  {
    return os << "LowVoltage(netMonitoringOffsets: " << lowVoltage.netMonitoringOffsets << ", "
              << "netDriverOffsets: " << lowVoltage.netDriverOffsets << ")";
  }
};

}  // namespace march
#endif  // MARCH_HARDWARE_LOW_VOLTAGE_H
