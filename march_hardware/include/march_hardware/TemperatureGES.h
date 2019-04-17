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
  int temperatureByteOffset;

public:
  TemperatureGES(int slaveIndex, int temperatureByteOffset);

  TemperatureGES()
  {
    temperatureByteOffset = -1;
    slaveIndex = -1;
  };

  float getTemperature() override;

  /** @brief Override comparison operator */
  friend bool operator==(const TemperatureGES& lhs, const TemperatureGES& rhs)
  {
    return lhs.slaveIndex == rhs.slaveIndex && lhs.temperatureByteOffset == rhs.temperatureByteOffset;
  }
  /** @brief Override stream operator for clean printing */
  friend ::std::ostream& operator<<(std::ostream& os, const TemperatureGES& temperatureGes)
  {
    return os << "slaveIndex: " << temperatureGes.slaveIndex << ", "
              << "temperatureByteOffset: " << temperatureGes.temperatureByteOffset;
  }
};
}  // namespace march4cpp
#endif
