// Copyright 2019 Project March.
#ifndef MARCH_IV_NETDRIVEROFFSETS_H
#define MARCH_IV_NETDRIVEROFFSETS_H

class NetDriverOffsets
{
  int lowVoltageNetOnOff;
  int highVoltageNetOnOff;
  int highVoltageOvercurrentReset;
  int highVoltageEmergencySwitchOnOff;

public:
  NetDriverOffsets(int lowVoltageNetOnOff, int highVoltageNetOnOff, int highVoltageOvercurrentReset,
                   int highVoltageEmergencySwitchOnOff)
    : lowVoltageNetOnOff(lowVoltageNetOnOff)
    , highVoltageNetOnOff(highVoltageNetOnOff)
    , highVoltageOvercurrentReset(highVoltageOvercurrentReset)
    , highVoltageEmergencySwitchOnOff(highVoltageEmergencySwitchOnOff)
  {
  }

  NetDriverOffsets()
  {
    lowVoltageNetOnOff = -1;
    highVoltageNetOnOff = -1;
    highVoltageOvercurrentReset = -1;
  }

  int getLowVoltageNetOnOff() const
  {
    return lowVoltageNetOnOff;
  }

  int getHighVoltageNetOnOff() const
  {
    return highVoltageNetOnOff;
  }

  int getHighVoltageOvercurrentReset() const
  {
    return highVoltageOvercurrentReset;
  }

  int getHighVoltageEmergencySwitchOnOff() const
  {
    return highVoltageEmergencySwitchOnOff;
  }
};

#endif  // MARCH_IV_NETDRIVEROFFSETS_H
