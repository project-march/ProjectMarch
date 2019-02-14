#ifndef MARCH4CPP__TEMPERATURESENSOR_H
#define MARCH4CPP__TEMPERATURESENSOR_H

#include <stdint.h>

class TemperatureSensor
{
private:
  int slaveIndex;
  uint8_t byteOffset;

public:
  TemperatureSensor(int slaveIndex, uint8_t byteOffset);

  void initialize();

  float getTemperature();
  int getSlaveIndex();
};

#endif
