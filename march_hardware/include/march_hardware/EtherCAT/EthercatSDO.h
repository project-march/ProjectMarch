// Copyright 2019 Project March.
#ifndef MARCH_HARDWARE_ETHERCAT_ETHERCATSDO_H
#define MARCH_HARDWARE_ETHERCAT_ETHERCATSDO_H

#include <cstdint>

namespace march
{
int sdo_bit8_write(uint16_t slave, uint16_t index, uint8_t sub, uint8_t value);
int sdo_bit8_read(uint16_t slave, uint16_t index, uint8_t sub, int *val_size, uint32_t *value);
int sdo_bit16_write(uint16_t slave, uint16_t index, uint8_t sub, uint16_t value);
int sdo_bit16_read(uint16_t slave, uint16_t index, uint8_t sub, int *val_size, uint32_t *value);
int sdo_bit32_write(uint16_t slave, uint16_t index, uint8_t sub, uint32_t value);
int sdo_bit32_read(uint16_t slave, uint16_t index, uint8_t sub, int *val_size, uint32_t *value);

}  // namespace march

#endif  // MARCH_HARDWARE_ETHERCAT_ETHERCATSDO_H
