#ifndef MARCH4CPP__TEMPERATURESENSOR_H
#define MARCH4CPP__TEMPERATURESENSOR_H

#include <stdint.h>
namespace march4cpp
{
class TemperatureSensor
{
private:
  int slaveIndex;
  uint8_t byteOffset;

public:
  TemperatureSensor(int slaveIndex, uint8_t byteOffset);

  TemperatureSensor()
  {
    slaveIndex = -1;
  };

  void initialize();

  float getTemperature();

  int getSlaveIndex();
};
}
#endif
