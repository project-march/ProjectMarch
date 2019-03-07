// Copyright 2019 Project March.
#include "march_hardware/EtherCAT/EthercatIO.h"

#include <march_hardware/TemperatureSensor.h>

namespace march4cpp
{
TemperatureSensor::TemperatureSensor(int slaveIndex, int byteOffset) : Slave(slaveIndex)
{
  if (byteOffset < 0)
  {
    ROS_FATAL("Slave configuration error: byteOffset can not be negative.");
    throw ::std::invalid_argument("Slave configuration error: byteOffset can not be negative");
  }
  this->byteOffset = byteOffset;
}

void TemperatureSensor::writeInitialSDOs(int ecatCycleTime)
{
  // TODO(Martijn) initialize PDO/SDO settings here if necessary
}

float TemperatureSensor::getTemperature()
{
  // TODO(Martijn) read actual data from ethercat.
  union bit8 return_byte = get_input_bit8(static_cast<uint16>(slaveIndex), static_cast<uint8>(byteOffset));
  return static_cast<float>(return_byte.ui);
}

}  // namespace march4cpp
