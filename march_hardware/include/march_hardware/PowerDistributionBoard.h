// Copyright 2019 Project March.
#ifndef MARCH4CPP__POWERDISTRIBUTIONBOARD_H
#define MARCH4CPP__POWERDISTRIBUTIONBOARD_H

#include <stdint.h>
#include <march_hardware/Slave.h>
#include <march_hardware/BootShutdownOffsets.h>
#include <march_hardware/NetMonitorOffsets.h>
#include <march_hardware/NetDriverOffsets.h>

namespace march4cpp
{
class PowerDistributionBoard : public Slave
{
private:
  NetMonitorOffsets netMonitoringOffsets;
  NetDriverOffsets netDriverOffsets;
  BootShutdownOffsets bootShutdownOffsets;

  uint8 getLowVoltageNetsOperational();
  uint8 getHighVoltageNetsOperational();

public:
  PowerDistributionBoard(int slaveIndex, NetMonitorOffsets netMonitoringOffsets, NetDriverOffsets netDriverOffsets,
                         BootShutdownOffsets bootShutdownOffsets);

  PowerDistributionBoard(){};

  float getPowerDistributionBoardCurrent();
  float getLowVoltageNetCurrent(int netNumber);
  float getHighVoltageNetCurrent();
  bool getMasterShutdownRequested();
  bool getLowVoltageNetOperational(int netNumber);
  bool getHighVoltageOvercurrentTrigger(int netNumber);
  bool getEmergencyButtonTriggered();
  bool getHighVoltageNetOperational(int netNumber);

  void setMasterOk(bool isOk);
  void setMasterShutDownAllowed(bool isAllowed);
  void setLowVoltageNetOnOff(bool on, int netNumber);
  void setHighVoltageNetOnOff(bool on, int netNumber);
  void setHighVoltageOvercurrentReset(bool on, int netNumber);
  void setHighVoltageEmergencySwitchOnOff(bool on);

  /** @brief Override comparison operator */
  friend bool operator==(const PowerDistributionBoard& lhs, const PowerDistributionBoard& rhs)
  {
    return lhs.slaveIndex == rhs.slaveIndex && lhs.netMonitoringOffsets == rhs.netMonitoringOffsets;
  }
  /** @brief Override stream operator for clean printing */
  friend ::std::ostream& operator<<(std::ostream& os, const PowerDistributionBoard& powerDistributionBoard)
  {
    return os << "slaveIndex: " << powerDistributionBoard.slaveIndex << ", "
              << "netMonitoringOffsets: (" << powerDistributionBoard.netMonitoringOffsets << ") ";
  }
};
}  // namespace march4cpp
#endif
