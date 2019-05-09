// Copyright 2019 Project March.
#ifndef MARCH_IV_NETDRIVEROFFSETS_H
#define MARCH_IV_NETDRIVEROFFSETS_H

class NetDriverOffsets
{
  int lowVoltageNetOnOff;
  int highVoltageNetOnOff;
  int highVoltageEmergencySwitchOnOff;
  // overcurrent reset is not supported at this moment
//  int highVoltageOvercurrentReset;

public:
  NetDriverOffsets(int lowVoltageNetOnOff, int highVoltageNetOnOff,
                   int highVoltageEmergencySwitchOnOff)
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
};

#endif  // MARCH_IV_NETDRIVEROFFSETS_H
