// Copyright 2019 Project March.

#ifndef MARCH_HARDWARE_IMOTIONCUBETARGETSTATE_H
#define MARCH_HARDWARE_IMOTIONCUBETARGETSTATE_H

#include <string>
namespace march
{
class IMotionCubeTargetState
{
public:
  static const IMotionCubeTargetState SWITCH_ON_DISABLED;
  static const IMotionCubeTargetState READY_TO_SWITCH_ON;
  static const IMotionCubeTargetState SWITCHED_ON;
  static const IMotionCubeTargetState OPERATION_ENABLED;

private:
  ::std::string description;
  uint16_t controlWord;
  uint16_t stateMask;
  uint16_t state;

private:
  IMotionCubeTargetState(const ::std::string& description, uint16_t controlWord, uint16_t stateMask, uint16_t state)
    : description(description), controlWord(controlWord), stateMask(stateMask), state(state)
  {
  }

public:
  bool isReached(uint16_t statusWord) const
  {
    return (statusWord & stateMask) == state;
  }

  const std::string& getDescription() const
  {
    return this->description;
  }
  uint16_t getControlWord() const
  {
    return this->controlWord;
  }
  uint16_t getStateMask() const
  {
    return this->stateMask;
  }
  uint16_t getState() const
  {
    return this->state;
  }
};

}  // namespace march
#endif  // MARCH_HARDWARE_IMOTIONCUBETARGETSTATE_H
