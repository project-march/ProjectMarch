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

  ActuationMode() : value_(unknown)
  {
  }

  ActuationMode(Value value) : value_(value)
  {
  }

  explicit ActuationMode(const std::string& actuationMode)
  {
    if (actuationMode == "position")
    {
      this->value_ = position;
    }
    else if (actuationMode == "unknown")
    {
      this->value_ = unknown;
    }
    else if (actuationMode == "torque")
    {
      this->value_ = torque;
    }
    else
    {
      ROS_WARN("Actuation mode (%s) is not recognized, setting to unknown mode", actuationMode.c_str());
      this->value_ = ActuationMode::unknown;
    }
  }

  uint8_t toModeNumber()
  {
    switch (this->value_)
    {
      case position:
        return 8;
      case torque:
        return 10;
      default:
        return 0;
    }
  }

  int getValue() const
  {
    return this->value_;
  }

  bool operator==(ActuationMode::Value a) const
  {
    return this->value_ == a;
  }

  bool operator!=(ActuationMode::Value a) const
  {
    return this->value_ != a;
  }

  std::string toString() const
  {
    switch (this->value_)
    {
      case position:
        return "position";
      case torque:
        return "torque";
      default:
        ROS_WARN("Actuationmode (%i) is neither 'torque' or 'position", this->value_);
        return "unknown";
    }
  }

private:
  Value value_ = unknown;
};
}  // namespace march

#endif  // MARCH_HARDWARE_ACTUATIONMODE_H
