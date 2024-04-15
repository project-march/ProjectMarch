// Copyright 2019 Project March.
#include "march_hardware/temperature/temperature_ges.h"
#include "march_hardware/ethercat/pdo_types.h"

namespace march {
TemperatureGES::TemperatureGES(const Slave& slave, uint8_t byte_offset)
    : Slave(slave)
    , temperature_byte_offset_(byte_offset)
{
}

float TemperatureGES::getTemperature() const
{
    bit32 temperature = this->read32(this->temperature_byte_offset_);
    return temperature.f;
}
} // namespace march
