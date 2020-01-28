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
  int stateMask;
  int state;

private:
  IMotionCubeTargetState(const ::std::string& description, uint16_t controlWord, int stateMask, int state)
    : description(description), controlWord(controlWord), stateMask(stateMask), state(state)
  {
  }

public:
  bool isReached(int statusWord)
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
  int getStateMask() const
  {
    return this->stateMask;
  }
  int getState() const
  {
    return this->state;
  }
};

}  // namespace march
#endif  // MARCH_HARDWARE_IMOTIONCUBETARGETSTATE_H
