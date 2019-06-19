// Copyright 2019 Project March.
#ifndef MARCH_IV_NETDRIVEROFFSETS_H
#define MARCH_IV_NETDRIVEROFFSETS_H

#include <ros/ros.h>

class NetDriverOffsets
{
  int lowVoltageNetOnOff;
  int highVoltageNetOnOff;
  int allHighVoltageOnOff;

public:
  NetDriverOffsets(int lowVoltageNetOnOff, int highVoltageNetOnOff, int allHighVoltageOnOff)
    : lowVoltageNetOnOff(lowVoltageNetOnOff)
    , highVoltageNetOnOff(highVoltageNetOnOff)
    , allHighVoltageOnOff(allHighVoltageOnOff)
  {
    if (lowVoltageNetOnOff < 0 || highVoltageNetOnOff < 0 || allHighVoltageOnOff < 0)
    {
      ROS_ERROR("Negative byte offset not possible");
      throw std::runtime_error("Negative byte offset not possible");
    }
  }

  NetDriverOffsets()
  {
    lowVoltageNetOnOff = -1;
    highVoltageNetOnOff = -1;
    allHighVoltageOnOff = -1;
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

  int getAllHighVoltageOnOff() const
  {
    if (allHighVoltageOnOff == -1)
    {
      ROS_FATAL("allHighVoltageOnOffOffset is -1");
      throw std::runtime_error("allHighVoltageOnOffOffset is -1");
    }
    return allHighVoltageOnOff;
  }

  /** @brief Override comparison operator */
  friend bool operator==(const NetDriverOffsets& lhs, const NetDriverOffsets& rhs)
  {
    return lhs.lowVoltageNetOnOff == rhs.lowVoltageNetOnOff && lhs.highVoltageNetOnOff == rhs.highVoltageNetOnOff &&
           lhs.allHighVoltageOnOff == rhs.allHighVoltageOnOff;
  }

  /** @brief Override stream operator for clean printing */
  friend ::std::ostream& operator<<(std::ostream& os, const NetDriverOffsets& netDriverOffsets)
  {
    return os << "NetDriverOffsets(lowVoltageNetOnOff: " << netDriverOffsets.lowVoltageNetOnOff << ", "
              << "highVoltageNetOnOff: " << netDriverOffsets.highVoltageNetOnOff << ", "
              << "allHighVoltageOnOff: " << netDriverOffsets.allHighVoltageOnOff << ")";
  }
};

#endif  // MARCH_IV_NETDRIVEROFFSETS_H
