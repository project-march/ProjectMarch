// Copyright 2019 Project March.

#ifndef MARCH_IV_POWERNETTYPE_H
#define MARCH_IV_POWERNETTYPE_H

#include <ros/package.h>

class PowerNetType {
public:
  enum Value : int { high_voltage = 0, low_voltage = 1 };

  PowerNetType() = default;

  explicit PowerNetType(const std::string &name) {
    if (name == "high_voltage") {
      this->value = high_voltage;
    } else if (name == "low_voltage") {
      this->value = low_voltage;
    } else {
      ROS_ASSERT_MSG(false, "Unknown power net type %s", name.c_str());
    }
  }

  bool operator==(PowerNetType a) const { return value == a.value; }
  bool operator!=(PowerNetType a) const { return value != a.value; }

  bool operator==(int a) const { return value == a; }
  bool operator!=(int a) const { return value != a; }

private:
  Value value;
};

#endif // MARCH_IV_POWERNETTYPE_H
