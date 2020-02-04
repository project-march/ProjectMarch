// Copyright 2019 Project March.

#ifndef MARCH_HARDWARE_ACTUATIONMODE_H
#define MARCH_HARDWARE_ACTUATIONMODE_H
#include <string>
#include <ros/console.h>

namespace march
{
class ActuationMode
{
public:
  enum Value : int
  {
    position,
    torque,
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
    else if (actuationMode == "torque")
    {
      this->value = torque;
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
    else if (value == torque)
    {
      return 10;
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
      case torque:
        return "torque";
      default:
        ROS_WARN("Actuationmode (%i) is neither 'torque' or 'position", value);
        return "unknown";
    }
  }

private:
  Value value = unknown;
};
}  // namespace march

#endif  // MARCH_HARDWARE_ACTUATIONMODE_H
