#ifndef MARCH4CPP__SDO_H
#define MARCH4CPP__SDO_H

#include <stdint.h>

namespace march4cpp
{

int sdo_bit8(int slave, uint32_t index, uint8_t sub, uint8_t value);
int sdo_bit16(int slave, uint32_t index, uint8_t sub, uint16_t value);
int sdo_bit32(int slave, uint32_t index, uint8_t sub, uint32_t value);

}  // namespace march4cpp
#endif  // MARCH4CPP__SDO_H