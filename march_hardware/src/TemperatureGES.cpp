// Copyright 2019 Project March.
#include "march_hardware/EtherCAT/pdo_types.h"

#include <march_hardware/TemperatureGES.h>

namespace march
{
TemperatureGES::TemperatureGES(Slave slave, uint8_t byte_offset) : Slave(slave), temperature_byte_offset_(byte_offset)
{
}

float TemperatureGES::getTemperature() const
{
  bit32 temperature = this->read32(this->getSlaveIndex(), this->temperature_byte_offset_);
  return temperature.f;
}

}  // namespace march
