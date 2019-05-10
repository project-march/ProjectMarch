// Copyright 2019 Project March.
#ifndef MARCH_IV_NETDRIVEROFFSETS_H
#define MARCH_IV_NETDRIVEROFFSETS_H

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
  }

  NetDriverOffsets()
  {
    lowVoltageNetOnOff = -1;
    highVoltageNetOnOff = -1;
  }

  int getLowVoltageNetOnOff() const
  {
    return lowVoltageNetOnOff;
  }

  int getHighVoltageNetOnOff() const
  {
    return highVoltageNetOnOff;
  }

  int getHighVoltageEmergencySwitchOnOff() const
  {
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
    return os << "NetDriverOffsets( lowVoltageNetOnOff: " << netDriverOffsets.lowVoltageNetOnOff << ", "
              << "highVoltageNetOnOff: " << netDriverOffsets.highVoltageNetOnOff << ", "
              << "highVoltageEmergencySwitchOnOff: " << netDriverOffsets.highVoltageEmergencySwitchOnOff << ")";
  }
};

#endif  // MARCH_IV_NETDRIVEROFFSETS_H
