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

  void initialize() override;

  float getTemperature() override;
};
}
#endif
