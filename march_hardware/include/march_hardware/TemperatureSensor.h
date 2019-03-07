#ifndef MARCH4CPP__TEMPERATURESENSOR_H
#define MARCH4CPP__TEMPERATURESENSOR_H

#include <stdint.h>
#include <march_hardware/Slave.h>

namespace march4cpp
{
class TemperatureSensor : public Slave
{
private:
  int byteOffset;

public:
  TemperatureSensor(int slaveIndex, int byteOffset);

  TemperatureSensor()
  {
    byteOffset = -1;
    slaveIndex = -1;
  };

  void writeInitialSDOs(int ecatCycleTime) override;

  float getTemperature();
};
}
#endif
