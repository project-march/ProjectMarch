// Copyright 2019 Project March.
#ifndef MARCH_HARDWARE_HIGHVOLTAGE_H
#define MARCH_HARDWARE_HIGHVOLTAGE_H
#include "march_hardware/EtherCAT/pdo_interface.h"
#include "march_hardware/NetDriverOffsets.h"
#include "march_hardware/NetMonitorOffsets.h"

#include <cstdint>
#include <iostream>

namespace march
{
class HighVoltage
{
private:
  PdoInterface& pdo_;
  int slaveIndex;
  NetMonitorOffsets netMonitoringOffsets;
  NetDriverOffsets netDriverOffsets;

  uint8_t getNetsOperational();

public:
  HighVoltage(PdoInterface& pdo, int slaveIndex, NetMonitorOffsets netMonitoringOffsets, NetDriverOffsets netDriverOffsets);

  float getNetCurrent();
  bool getNetOperational(int netNumber);
  bool getOvercurrentTrigger(int netNumber);
  bool getHighVoltageEnabled();
  void setNetOnOff(bool on, int netNumber);
  void enableDisableHighVoltage(bool enable);

  /** @brief Override comparison operator */
  friend bool operator==(const HighVoltage& lhs, const HighVoltage& rhs)
  {
    return lhs.slaveIndex == rhs.slaveIndex && lhs.netDriverOffsets == rhs.netDriverOffsets &&
           lhs.netMonitoringOffsets == rhs.netMonitoringOffsets;
  }

  /** @brief Override stream operator for clean printing */
  friend ::std::ostream& operator<<(std::ostream& os, const HighVoltage& highVoltage)
  {
    return os << "HighVoltage(slaveIndex: " << highVoltage.slaveIndex << ", "
              << "netMonitoringOffsets: " << highVoltage.netMonitoringOffsets << ", "
              << "netDriverOffsets: " << highVoltage.netDriverOffsets << ")";
  }
};

}  // namespace march
#endif  // MARCH_HARDWARE_HIGHVOLTAGE_H
