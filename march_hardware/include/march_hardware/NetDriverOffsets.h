// Copyright 2019 Project March.
#ifndef MARCH_IV_NETDRIVEROFFSETS_H
#define MARCH_IV_NETDRIVEROFFSETS_H

#include <ros/ros.h>

class NetDriverOffsets
{
  int lowVoltageNetOnOff;
  int highVoltageNetOnOff;
  int highVoltageNetEnableDisable;

public:
  NetDriverOffsets(int lowVoltageNetOnOff, int highVoltageNetOnOff, int highVoltageNetEnableDisable)
    : lowVoltageNetOnOff(lowVoltageNetOnOff)
    , highVoltageNetOnOff(highVoltageNetOnOff)
    , highVoltageNetEnableDisable(highVoltageNetEnableDisable)
  {
    if (lowVoltageNetOnOff < 0 || highVoltageNetOnOff < 0 || highVoltageNetEnableDisable < 0)
    {
      ROS_ERROR("Negative byte offset not possible");
      throw std::runtime_error("Negative byte offset not possible");
    }
  }

  NetDriverOffsets()
  {
    lowVoltageNetOnOff = -1;
    highVoltageNetOnOff = -1;
    highVoltageNetEnableDisable = -1;
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

  int getHighVoltageEnableDisable() const
  {
    if (highVoltageNetEnableDisable == -1)
    {
      ROS_FATAL("highVoltageNetEnableDisable is -1");
      throw std::runtime_error("highVoltageNetEnableDisable is -1");
    }
    return highVoltageNetEnableDisable;
  }

  /** @brief Override comparison operator */
  friend bool operator==(const NetDriverOffsets& lhs, const NetDriverOffsets& rhs)
  {
    return lhs.lowVoltageNetOnOff == rhs.lowVoltageNetOnOff && lhs.highVoltageNetOnOff == rhs.highVoltageNetOnOff &&
           lhs.highVoltageNetEnableDisable == rhs.highVoltageNetEnableDisable;
  }

  /** @brief Override stream operator for clean printing */
  friend ::std::ostream& operator<<(std::ostream& os, const NetDriverOffsets& netDriverOffsets)
  {
    return os << "NetDriverOffsets(lowVoltageNetOnOff: " << netDriverOffsets.lowVoltageNetOnOff << ", "
              << "highVoltageNetOnOff: " << netDriverOffsets.highVoltageNetOnOff << ", "
              << "highVoltageNetEnableDisable: " << netDriverOffsets.highVoltageNetEnableDisable << ")";
  }
};

#endif  // MARCH_IV_NETDRIVEROFFSETS_H
