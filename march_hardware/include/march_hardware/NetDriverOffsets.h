// Copyright 2019 Project March.
#ifndef MARCH_IV_NETDRIVEROFFSETS_H
#define MARCH_IV_NETDRIVEROFFSETS_H

#include <ros/ros.h>

class NetDriverOffsets
{
  int lowVoltageNetOnOff;
  int highVoltageNetOnOff;
  int highVoltageEmergencySwitchOnOff;

public:
  NetDriverOffsets(int lowVoltageNetOnOff, int highVoltageNetOnOff, int highVoltageEmergencySwitchOnOff)
    : lowVoltageNetOnOff(lowVoltageNetOnOff)
    , highVoltageNetOnOff(highVoltageNetOnOff)
    , highVoltageEmergencySwitchOnOff(highVoltageEmergencySwitchOnOff)
  {
    if (lowVoltageNetOnOff < 0 || highVoltageNetOnOff < 0 || highVoltageEmergencySwitchOnOff < 0)
    {
      ROS_ERROR("Negative byte offset not possible");
      throw std::runtime_error("Negative byte offset not possible");
    }
  }

  NetDriverOffsets()
  {
    lowVoltageNetOnOff = -1;
    highVoltageNetOnOff = -1;
    highVoltageEmergencySwitchOnOff = -1;
  }

  int getLowVoltageNetOnOff() const
  {
    if (lowVoltageNetOnOff == -1)
    {
      ROS_FATAL("lowVoltageNetOnOffOffset is -1");
      throw std::runtime_error("lowVoltageNetOnOffOffset is -1");
    }
    return lowVoltageNetOnOff;
  }

  int getHighVoltageNetOnOff() const
  {
    if (highVoltageNetOnOff == -1)
    {
      ROS_FATAL("highVoltageNetOnOffOffset is -1");
      throw std::runtime_error("highVoltageNetOnOffOffset is -1");
    }
    return highVoltageNetOnOff;
  }

  int getHighVoltageEmergencySwitchOnOff() const
  {
    if (highVoltageEmergencySwitchOnOff == -1)
    {
      ROS_FATAL("highVoltageEmergencySwitchOnOffOffset is -1");
      throw std::runtime_error("highVoltageEmergencySwitchOnOffOffset is -1");
    }
    return highVoltageEmergencySwitchOnOff;
  }

  /** @brief Override comparison operator */
  friend bool operator==(const NetDriverOffsets& lhs, const NetDriverOffsets& rhs)
  {
    return lhs.lowVoltageNetOnOff == rhs.lowVoltageNetOnOff && lhs.highVoltageNetOnOff == rhs.highVoltageNetOnOff &&
           lhs.highVoltageEmergencySwitchOnOff == rhs.highVoltageEmergencySwitchOnOff;
  }

  /** @brief Override stream operator for clean printing */
  friend ::std::ostream& operator<<(std::ostream& os, const NetDriverOffsets& netDriverOffsets)
  {
    return os << "NetDriverOffsets(lowVoltageNetOnOff: " << netDriverOffsets.lowVoltageNetOnOff << ", "
              << "highVoltageNetOnOff: " << netDriverOffsets.highVoltageNetOnOff << ", "
              << "highVoltageEmergencySwitchOnOff: " << netDriverOffsets.highVoltageEmergencySwitchOnOff << ")";
  }
};

#endif  // MARCH_IV_NETDRIVEROFFSETS_H
