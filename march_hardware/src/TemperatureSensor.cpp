#include "march_hardware/EtherCAT/EthercatIO.h"

#include <march_hardware/TemperatureSensor.h>

TemperatureSensor::TemperatureSensor(int slaveIndex, uint8_t byteOffset)
{
  this->slaveIndex = slaveIndex;
  this->byteOffset = byteOffset;
}

void TemperatureSensor::initialize()
{
  //  TODO(Martijn) initialize PDO/SDO settings here.
}

float TemperatureSensor::getTemperature()
{
  //  TODO(Martijn) read actual data from ethercat.
  union bit8 return_byte = get_input_bit8(slaveIndex, byteOffset);
  return (float)return_byte.ui;
}

int TemperatureSensor::getSlaveIndex()
{
  return slaveIndex;
}
