// Copyright 2019 Project March.
#ifndef MARCH4CPP__POWERDISTRIBUTIONBOARD_H
#define MARCH4CPP__POWERDISTRIBUTIONBOARD_H

#include <stdint.h>
#include <march_hardware/Slave.h>
#include <march_hardware/StateOffsets.h>
#include <march_hardware/NetMonitoringOffsets.h>

namespace march4cpp
{
class PowerDistributionBoard : public Slave
{
private:
  NetMonitoringOffsets netMonitoringOffsets;
  StateOffsets stateOffsets;

public:
  PowerDistributionBoard(int slaveIndex, NetMonitoringOffsets currentOffsets, StateOffsets stateOffsets);

  PowerDistributionBoard(){};

  float getPowerDistributionBoardCurrent();
  float getLowVoltageNet1Current();
  float getLowVoltageNet2Current();
  float getHighVoltageNetCurrent();
    bool getMasterShutdownRequested();
    bool getLowVoltageNet1Operational();
    bool getLowVoltageNet2Operational();

  void setMasterOk(bool isOk);
  void setMasterShutDownAllowed(bool isAllowed);

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
