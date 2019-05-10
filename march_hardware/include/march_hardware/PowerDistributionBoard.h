// Copyright 2019 Project March.
#ifndef MARCH4CPP__POWERDISTRIBUTIONBOARD_H
#define MARCH4CPP__POWERDISTRIBUTIONBOARD_H

#include <stdint.h>
#include <march_hardware/Slave.h>
#include <march_hardware/BootShutdownOffsets.h>
#include <march_hardware/NetMonitorOffsets.h>
#include <march_hardware/NetDriverOffsets.h>
#include <march_hardware/HighVoltage.h>
#include <march_hardware/LowVoltage.h>
#include <march_hardware/EtherCAT/EthercatIO.h>

namespace march4cpp
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
  PowerDistributionBoard(int slaveIndex, NetMonitorOffsets netMonitoringOffsets, NetDriverOffsets netDriverOffsets,
                         BootShutdownOffsets bootShutdownOffsets);

  PowerDistributionBoard() = default;

  float getPowerDistributionBoardCurrent();
  bool getMasterShutdownRequested();
  void setMasterOnline();
  void setMasterShutDownAllowed(bool isAllowed);

  HighVoltage getHighVoltage();
  LowVoltage getLowVoltage();

  /** @brief Override comparison operator */
  friend bool operator==(const PowerDistributionBoard& lhs, const PowerDistributionBoard& rhs)
  {
    return lhs.slaveIndex == rhs.slaveIndex && lhs.netMonitoringOffsets == rhs.netMonitoringOffsets &&
           lhs.netDriverOffsets == rhs.netDriverOffsets && lhs.lowVoltage == rhs.lowVoltage &&
           lhs.highVoltage == rhs.highVoltage;
  }
  /** @brief Override stream operator for clean printing */
  friend ::std::ostream& operator<<(std::ostream& os, const PowerDistributionBoard& powerDistributionBoard)
  {
    return os << "PowerDistributionBoard(slaveIndex: " << powerDistributionBoard.slaveIndex << ", "
              << "masterOnlineToggle: " << powerDistributionBoard.masterOnlineToggle << ", "
              << powerDistributionBoard.netMonitoringOffsets << ", "
              << powerDistributionBoard.netDriverOffsets << ", "
              << powerDistributionBoard.highVoltage << ", "
              << powerDistributionBoard.lowVoltage << ")";
  }
};
}  // namespace march4cpp
#endif
