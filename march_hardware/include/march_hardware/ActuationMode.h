// Copyright 2019 Project March.

#ifndef MARCH_HARDWARE_INTERFACE_ACTUATIONMODE_H
#define MARCH_HARDWARE_INTERFACE_ACTUATIONMODE_H

#include <ros/console.h>

namespace march4cpp
{
class ActuationMode
{
public:
  enum Value : int
  {
    position,
    unknown,
  };

  ActuationMode()
  {
    this->value = unknown;
  };

  explicit ActuationMode(const std::string& actuationMode)
  {
    if (actuationMode == "position")
    {
      this->value = position;
    }
    else if (actuationMode == "unknown")
    {
      this->value = unknown;
    }
    else
    {
      ROS_WARN("Actuation mode (%s) is not recognized, setting to unknown mode", actuationMode.c_str());
      this->value = ActuationMode::unknown;
    }
  }

  int toModeNumber()
  {
    if (value == position)
    {
      return 8;
    }
  }

  int getValue() const
  {
    return value;
  }

  bool operator==(ActuationMode::Value a) const
  {
    return value == a;
  }

  bool operator!=(ActuationMode::Value a) const
  {
    return value != a;
  }

  std::string toString() const
  {
    switch (this->value)
    {
      case position:
        return "position";
      default:
        ROS_WARN("Actuationmode (%i) is neither 'torque' or 'position", value);
        return "unknown";
    }
  }

private:
  Value value = unknown;
};
}  // namespace march4cpp

#endif  // MARCH_HARDWARE_INTERFACE_ACTUATIONMODE_H
