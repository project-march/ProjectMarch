// Copyright 2019 Project March.
#ifndef MARCH_HARDWARE_ETHERCAT_ETHERCATSDO_H
#define MARCH_HARDWARE_ETHERCAT_ETHERCATSDO_H

#include <cstdint>

namespace march
{
int sdo_bit8(uint16_t slave, uint16_t index, uint8_t sub, uint8_t value);
int sdo_bit16(uint16_t slave, uint16_t index, uint8_t sub, uint16_t value);
int sdo_bit32(uint16_t slave, uint16_t index, uint8_t sub, uint32_t value);

}  // namespace march

#endif  // MARCH_HARDWARE_ETHERCAT_ETHERCATSDO_H
