// Copyright 2019 Project March.
#ifndef MARCH_FAKE_SENSOR_DATA_HIGHVOLTAGE_H
#define MARCH_FAKE_SENSOR_DATA_HIGHVOLTAGE_H

#include <ros/ros.h>

#include <march_hardware/EtherCAT/EthercatIO.h>
#include <march_hardware/NetDriverOffsets.h>
#include <march_hardware/NetMonitorOffsets.h>

namespace march4cpp
{
class HighVoltage
{
private:
  int slaveIndex;
  NetMonitorOffsets netMonitoringOffsets;
  NetDriverOffsets netDriverOffsets;

  uint8 getHighVoltageNetsOperational();

public:
  HighVoltage(int slaveIndex, NetMonitorOffsets netMonitoringOffsets, NetDriverOffsets netDriverOffsets);
  HighVoltage();

  float getNetCurrent();
  bool getNetOperational(int netNumber);
  bool getOvercurrentTrigger(int netNumber);
  bool getEmergencyButtonTriggered();
  void setHighVoltageNetOnOff(bool on, int netNumber);
  void setHighVoltageEmergencySwitchOnOff(bool on);

  /** @brief Override comparison operator */
  friend bool operator==(const HighVoltage& lhs, const HighVoltage& rhs)
  {
    return lhs.slaveIndex == rhs.slaveIndex;
  }

  /** @brief Override stream operator for clean printing */
  friend ::std::ostream& operator<<(std::ostream& os, const HighVoltage& highVoltage)
  {
    return os << "HighVoltage(slaveIndex: " << highVoltage.slaveIndex << ")";
  }

};

}  // namespace march4cpp
#endif  // MARCH_FAKE_SENSOR_DATA_HIGHVOLTAGE_H
