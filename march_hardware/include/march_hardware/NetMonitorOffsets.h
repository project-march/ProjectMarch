// Copyright 2019 Project March.
#ifndef MARCH_IV_NETMONITOROFFSETS_H
#define MARCH_IV_NETMONITOROFFSETS_H

#include <ostream>

class NetMonitorOffsets
{
  int powerDistributionBoardCurrent;
  int lowVoltageNet1Current;
  int lowVoltageNet2Current;
  int highVoltageNetCurrent;
  int lowVoltageState;
  int highVoltageOvercurrentTrigger;
  int emergencyButtonTriggered;
  int highVoltageState;

public:
  NetMonitorOffsets(int powerDistributionBoardCurrentByteOffset, int lowVoltageNet1CurrentByteOffset,
                    int lowVoltageNet2CurrentByteOffset, int highVoltageNetCurrentByteOffset,
                    int lowVoltageStateByteOffset, int highVoltageOvercurrentTriggerByteOffset,
                    int emergencyButtonTriggeredByteOffset, int highVoltageStateByteOffset)
    : powerDistributionBoardCurrent(powerDistributionBoardCurrentByteOffset)
    , lowVoltageNet1Current(lowVoltageNet1CurrentByteOffset)
    , lowVoltageNet2Current(lowVoltageNet2CurrentByteOffset)
    , highVoltageNetCurrent(highVoltageNetCurrentByteOffset)
    , lowVoltageState(lowVoltageStateByteOffset)
    , highVoltageOvercurrentTrigger(highVoltageOvercurrentTriggerByteOffset)
    , emergencyButtonTriggered(emergencyButtonTriggeredByteOffset)
    , highVoltageState(highVoltageStateByteOffset)
  {
  }

  NetMonitorOffsets()
  {
    powerDistributionBoardCurrent = -1;
    lowVoltageNet1Current = -1;
    lowVoltageNet2Current = -1;
    highVoltageNetCurrent = -1;
    lowVoltageState = -1;
    highVoltageOvercurrentTrigger = -1;
    emergencyButtonTriggered = -1;
    highVoltageState = -1;
  }

  int getHighVoltageState() const
  {
    return highVoltageState;
  }

  int getPowerDistributionBoardCurrent() const
  {
    return powerDistributionBoardCurrent;
  }

  int getHighVoltageOvercurrentTrigger() const
  {
    return highVoltageOvercurrentTrigger;
  }

  int getEmergencyButtonTriggered() const
  {
    return emergencyButtonTriggered;
  }

  int getLowVoltageNetCurrent(int netNumber) const
  {
    if (netNumber == 1)
    {
      return lowVoltageNet1Current;
    }
    else if (netNumber == 2)
    {
      return lowVoltageNet2Current;
    }
    else
    {
      ROS_FATAL("Can't get the byte offset for low voltage net current %d, there are only 2 low voltage nets",
                netNumber);
      throw std::exception();
    }
  }

  int getHighVoltageNetCurrent() const
  {
    return highVoltageNetCurrent;
  }

  int getLowVoltageState() const
  {
    return lowVoltageState;
  }

  //  @TODO(TIM) Add all attributes:
  /** @brief Override comparison operator */
  friend bool operator==(const NetMonitorOffsets& lhs, const NetMonitorOffsets& rhs)
  {
    return lhs.powerDistributionBoardCurrent == rhs.powerDistributionBoardCurrent &&
           lhs.lowVoltageNet1Current == rhs.lowVoltageNet1Current &&
           lhs.lowVoltageNet2Current == rhs.lowVoltageNet2Current &&
           lhs.highVoltageNetCurrent == rhs.highVoltageNetCurrent;
  }
  /** @brief Override stream operator for clean printing */
  friend ::std::ostream& operator<<(std::ostream& os, const NetMonitorOffsets& currentOffsets)
  {
    return os << "powerDistributionBoardCurrent: " << currentOffsets.powerDistributionBoardCurrent << ", "
              << "lowVoltageNet1Current: " << currentOffsets.lowVoltageNet1Current << ", "
              << "lowVoltageNet2Current: " << currentOffsets.lowVoltageNet2Current << ", "
              << "highVoltageNetCurrent: " << currentOffsets.highVoltageNetCurrent;
  }
};

#endif  // MARCH_IV_NETMONITOROFFSETS_H