// Copyright 2019 Project March.
#ifndef MARCH_IV_NETMONITOROFFSETS_H
#define MARCH_IV_NETMONITOROFFSETS_H

#include <ostream>

class NetMonitorOffsets
{
  // TODO(TIM) remove Offset part!
  int powerDistributionBoardCurrentByteOffset;
  int lowVoltageNet1CurrentByteOffset;
  int lowVoltageNet2CurrentByteOffset;
  int highVoltageNetCurrentByteOffset;
  int lowVoltageStateByteOffset;
  int highVoltageOvercurrentTriggerByteOffset;
  int emergencyButtonTriggeredByteOffset;
  int highVoltageStateByteOffset;

public:
  NetMonitorOffsets(int powerDistributionBoardCurrentByteOffset, int lowVoltageNet1CurrentByteOffset,
                    int lowVoltageNet2CurrentByteOffset, int highVoltageNetCurrentByteOffset,
                    int lowVoltageStateByteOffset, int highVoltageOvercurrentTriggerByteOffset,
                    int emergencyButtonTriggeredByteOffset, int highVoltageStateByteOffset)
    : powerDistributionBoardCurrentByteOffset(powerDistributionBoardCurrentByteOffset)
    , lowVoltageNet1CurrentByteOffset(lowVoltageNet1CurrentByteOffset)
    , lowVoltageNet2CurrentByteOffset(lowVoltageNet2CurrentByteOffset)
    , highVoltageNetCurrentByteOffset(highVoltageNetCurrentByteOffset)
    , lowVoltageStateByteOffset(lowVoltageStateByteOffset)
    , highVoltageOvercurrentTriggerByteOffset(highVoltageOvercurrentTriggerByteOffset)
    , emergencyButtonTriggeredByteOffset(emergencyButtonTriggeredByteOffset)
    , highVoltageStateByteOffset(highVoltageStateByteOffset)
  {
  }

  NetMonitorOffsets()
  {
    powerDistributionBoardCurrentByteOffset = -1;
    lowVoltageNet1CurrentByteOffset = -1;
    lowVoltageNet2CurrentByteOffset = -1;
    highVoltageNetCurrentByteOffset = -1;
    lowVoltageStateByteOffset = -1;
    highVoltageOvercurrentTriggerByteOffset = -1;
    emergencyButtonTriggeredByteOffset = -1;
    highVoltageStateByteOffset = -1;
  }

  int getHighVoltageStateByteOffset() const
  {
    return highVoltageStateByteOffset;
  }

  int getPowerDistributionBoardCurrentByteOffset() const
  {
    return powerDistributionBoardCurrentByteOffset;
  }

  int getHighVoltageOvercurrentTriggerByteOffset() const
  {
    return highVoltageOvercurrentTriggerByteOffset;
  }

  int getEmergencyButtonTriggeredByteOffset() const
  {
    return emergencyButtonTriggeredByteOffset;
  }

  int getLowVoltageNetCurrentByteOffset(int netNumber) const
  {
    if (netNumber == 1)
    {
      return lowVoltageNet1CurrentByteOffset;
    }
    else if (netNumber == 2)
    {
      return lowVoltageNet2CurrentByteOffset;
    }
    else
    {
      ROS_FATAL("Can't get the byte offset for low voltage net current %d, there are only 2 low voltage nets",
                netNumber);
      throw std::exception();
    }
  }

  int getHighVoltageNetCurrentByteOffset() const
  {
    return highVoltageNetCurrentByteOffset;
  }

  int getLowVoltageStateByteOffset() const
  {
    return lowVoltageStateByteOffset;
  }

  //  @TODO(TIM) Add all attributes:
  /** @brief Override comparison operator */
  friend bool operator==(const NetMonitorOffsets& lhs, const NetMonitorOffsets& rhs)
  {
    return lhs.powerDistributionBoardCurrentByteOffset == rhs.powerDistributionBoardCurrentByteOffset &&
           lhs.lowVoltageNet1CurrentByteOffset == rhs.lowVoltageNet1CurrentByteOffset &&
           lhs.lowVoltageNet2CurrentByteOffset == rhs.lowVoltageNet2CurrentByteOffset &&
           lhs.highVoltageNetCurrentByteOffset == rhs.highVoltageNetCurrentByteOffset;
  }
  /** @brief Override stream operator for clean printing */
  friend ::std::ostream& operator<<(std::ostream& os, const NetMonitorOffsets& currentOffsets)
  {
    return os << "powerDistributionBoardCurrentByteOffset: " << currentOffsets.powerDistributionBoardCurrentByteOffset
              << ", "
              << "lowVoltageNet1CurrentByteOffset: " << currentOffsets.lowVoltageNet1CurrentByteOffset << ", "
              << "lowVoltageNet2CurrentByteOffset: " << currentOffsets.lowVoltageNet2CurrentByteOffset << ", "
              << "highVoltageNetCurrentByteOffset: " << currentOffsets.highVoltageNetCurrentByteOffset;
  }
};

#endif  // MARCH_IV_NETMONITOROFFSETS_H