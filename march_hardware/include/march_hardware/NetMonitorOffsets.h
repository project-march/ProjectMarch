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
    if(highVoltageState == -1){
      ROS_FATAL("highVoltageStateOffset is -1");
      throw std::runtime_error("highVoltageStateOffset is -1");
    }
    return highVoltageState;
  }

  int getPowerDistributionBoardCurrent() const
  {
    if(powerDistributionBoardCurrent == -1){
      ROS_FATAL("powerDistributionBoardCurrent is -1");
      throw std::runtime_error("powerDistributionBoardCurrent is -1");
    }
    return powerDistributionBoardCurrent;
  }

  int getHighVoltageOvercurrentTrigger() const
  {
    if(highVoltageOvercurrentTrigger == -1){
      ROS_FATAL("highVoltageOvercurrentTrigger is -1");
      throw std::runtime_error("highVoltageOvercurrentTrigger is -1");
    }
    return highVoltageOvercurrentTrigger;
  }

  int getEmergencyButtonTriggered() const
  {
    if(emergencyButtonTriggered == -1){
      ROS_FATAL("emergencyButtonTriggeredOffset is -1");
      throw std::runtime_error("emergencyButtonTriggeredOffset is -1");
    }
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
    if(highVoltageNetCurrent == -1){
      ROS_FATAL("highVoltageNetCurrentOffset is -1");
      throw std::runtime_error("highVoltageNetCurrentOffset is -1");
    }
    return highVoltageNetCurrent;
  }

  int getLowVoltageState() const
  {
    if(lowVoltageState == -1){
      ROS_FATAL("lowVoltageStateOffset is -1");
      throw std::runtime_error("lowVoltageStateOffset is -1");
    }
    return lowVoltageState;
  }

  /** @brief Override comparison operator */
  friend bool operator==(const NetMonitorOffsets& lhs, const NetMonitorOffsets& rhs)
  {
    return lhs.powerDistributionBoardCurrent == rhs.powerDistributionBoardCurrent &&
           lhs.lowVoltageNet1Current == rhs.lowVoltageNet1Current &&
           lhs.lowVoltageNet2Current == rhs.lowVoltageNet2Current &&
           lhs.highVoltageNetCurrent == rhs.highVoltageNetCurrent && lhs.lowVoltageState == rhs.lowVoltageState &&
           lhs.highVoltageOvercurrentTrigger == rhs.highVoltageOvercurrentTrigger &&
           lhs.emergencyButtonTriggered == rhs.emergencyButtonTriggered && lhs.highVoltageState == rhs.highVoltageState;
  }

  /** @brief Override stream operator for clean printing */
  friend ::std::ostream& operator<<(std::ostream& os, const NetMonitorOffsets& netMonitorOffsets)
  {
    return os << "NetMonitorOffsets(powerDistributionBoardCurrent: " << netMonitorOffsets.powerDistributionBoardCurrent
              << ", "
              << "lowVoltageNet1Current: " << netMonitorOffsets.lowVoltageNet1Current << ", "
              << "lowVoltageNet2Current: " << netMonitorOffsets.lowVoltageNet2Current << ", "
              << "highVoltageNetCurrent: " << netMonitorOffsets.highVoltageNetCurrent << ", "
              << "lowVoltageState: " << netMonitorOffsets.lowVoltageState << ", "
              << "highVoltageOvercurrentTrigger: " << netMonitorOffsets.highVoltageOvercurrentTrigger << ", "
              << "emergencyButtonTriggered: " << netMonitorOffsets.emergencyButtonTriggered << ", "
              << "highVoltageState: " << netMonitorOffsets.highVoltageState << ")";
  }
};

#endif  // MARCH_IV_NETMONITOROFFSETS_H