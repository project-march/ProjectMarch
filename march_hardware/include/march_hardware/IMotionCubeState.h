// Copyright 2019 Project March.

#ifndef MARCH_WS_IMOTIONCUBESTATE_H
#define MARCH_WS_IMOTIONCUBESTATE_H

namespace march4cpp
{
class IMCState
{
public:
  enum Value : int
  {
    notReadyToSwitchOn,
    switchOnDisabled,
    readyToSwitchOn,
    switchedOn,
    operationEnabled,
    quickStopActive,
    faultReactionActive,
    fault,
    unknown,
    none
  };

  IMCState() : value(none)
  {
  }

  std::string getString()
  {
    if (this->value == notReadyToSwitchOn)
    {
      return "Not Ready To Switch On";
    }
    else if (this->value == switchOnDisabled)
    {
      return "Switch On Disabled";
    }
    else if (this->value == readyToSwitchOn)
    {
      return "Ready to Switch On";
    }
    else if (this->value == switchedOn)
    {
      return "Switched On";
    }
    else if (this->value == operationEnabled)
    {
      return "Operation Enabled";
    }
    else if (this->value == quickStopActive)
    {
      return "Quick Stop Active";
    }
    else if (this->value == faultReactionActive)
    {
      return "Fault Reaction Active";
    }
    else if (this->value == fault)
    {
      return "Fault";
    }
    else
      return "Not in a recognized IMC state";
  }

  constexpr IMCState(Value aIMCState) : value(aIMCState)
  {
  }

  bool operator==(IMCState a) const
  {
    return value == a.value;
  }
  bool operator!=(IMCState a) const
  {
    return value != a.value;
  }

private:
  Value value;
};

struct IMotionCubeState
{
public:
  IMotionCubeState() = default;

  std::string statusWord;
  std::string detailedError;
  std::string motionError;
  IMCState state;
  std::string detailedErrorDescription;
  std::string motionErrorDescription;
  float motorCurrent;
  float motorVoltage;
};

}  // namespace march4cpp

#endif  // MARCH_WS_IMOTIONCUBESTATE_H
