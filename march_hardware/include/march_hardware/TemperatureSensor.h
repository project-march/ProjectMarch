#ifndef MARCH4CPP__TEMPERATURESENSOR_H
#define MARCH4CPP__TEMPERATURESENSOR_H

#include <stdint.h>
#include <march_hardware/Slave.h>

namespace march4cpp
{
class TemperatureSensor : public Slave
{
private:
  uint8_t byteOffset;

public:
  TemperatureSensor(int slaveIndex, uint8_t byteOffset);

  TemperatureSensor()
  {
    byteOffset = static_cast<uint8_t>(-1);
    slaveIndex = -1;
  };

  void initialize() override;

  float getTemperature();

};
}
#endif
