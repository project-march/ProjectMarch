// Copyright 2019 Project March.
#ifndef MARCH4CPP__TEMPERATURESENSOR_H
#define MARCH4CPP__TEMPERATURESENSOR_H

#include <stdint.h>
#include <march_hardware/Slave.h>
#include "TemperatureSensor.h"

namespace march4cpp
{
class TemperatureGES : public Slave, TemperatureSensor
{
private:
  int byteOffset;

public:
  TemperatureGES(int slaveIndex, int byteOffset);

  TemperatureGES()
  {
    byteOffset = -1;
    slaveIndex = -1;
  };

  float getTemperature() override;

  /** @brief Override comparison operator */
  friend bool operator==(const TemperatureGES& lhs, const TemperatureGES& rhs)
  {
    return lhs.slaveIndex == rhs.slaveIndex && lhs.byteOffset == rhs.byteOffset;
  }
  /** @brief Override stream operator for clean printing */
  friend ::std::ostream& operator<<(std::ostream& os, const TemperatureGES& temperatureGes)
  {
    return os << "slaveIndex: " << temperatureGes.slaveIndex << ", "
              << "byteOffset: " << temperatureGes.byteOffset;
  }
};
}  // namespace march4cpp
#endif
