// Copyright 2019 Project March.
#ifndef MARCH_HARDWARE_POWER_DISTRIBUTION_BOARD_H
#define MARCH_HARDWARE_POWER_DISTRIBUTION_BOARD_H
#include "boot_shutdown_offsets.h"
#include "march_hardware/communication/ethercat/slave.h"
#include "high_voltage.h"
#include "low_voltage.h"
#include "net_driver_offsets.h"
#include "net_monitor_offsets.h"

namespace march
{
class PowerDistributionBoard : public Slave
{
private:
  NetMonitorOffsets netMonitoringOffsets;
  NetDriverOffsets netDriverOffsets;
  BootShutdownOffsets bootShutdownOffsets;
  HighVoltage highVoltage;
  LowVoltage lowVoltage;
  bool masterOnlineToggle;

public:
  PowerDistributionBoard(const Slave& slave, NetMonitorOffsets netMonitoringOffsets, NetDriverOffsets netDriverOffsets,
                         BootShutdownOffsets bootShutdownOffsets);

  float getPowerDistributionBoardCurrent();
  bool getMasterShutdownRequested();
  void setMasterOnline();
  void setMasterShutDownAllowed(bool isAllowed);

  HighVoltage getHighVoltage();
  LowVoltage getLowVoltage();

  /** @brief Override comparison operator */
  friend bool operator==(const PowerDistributionBoard& lhs, const PowerDistributionBoard& rhs)
  {
    return lhs.getSlaveIndex() == rhs.getSlaveIndex() && lhs.netMonitoringOffsets == rhs.netMonitoringOffsets &&
           lhs.netDriverOffsets == rhs.netDriverOffsets && lhs.lowVoltage == rhs.lowVoltage &&
           lhs.highVoltage == rhs.highVoltage;
  }
  /** @brief Override stream operator for clean printing */
  friend ::std::ostream& operator<<(std::ostream& os, const PowerDistributionBoard& powerDistributionBoard)
  {
    return os << "PowerDistributionBoard(slaveIndex: " << powerDistributionBoard.getSlaveIndex() << ", "
              << "masterOnlineToggle: " << powerDistributionBoard.masterOnlineToggle << ", "
              << powerDistributionBoard.highVoltage << ", " << powerDistributionBoard.lowVoltage << ")";
  }
};
}  // namespace march
#endif  // MARCH_HARDWARE_POWER_DISTRIBUTION_BOARD_H
