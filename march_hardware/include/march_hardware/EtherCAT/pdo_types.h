// Copyright 2019 Project March.
#ifndef MARCH_HARDWARE_ETHERCAT_PDO_TYPES_H
#define MARCH_HARDWARE_ETHERCAT_PDO_TYPES_H
#include <cstdint>

namespace march
{
// struct used to easily get specific bytes from a 32 bit variable
struct packed_bit32
{
  uint8_t b0;
  uint8_t b1;
  uint8_t b2;
  uint8_t b3;
};

// union used for int32, uint32 and float
union bit32
{
  int32_t i;
  uint32_t ui;
  float f;
  packed_bit32 p;
};

// struct used to easily get specific bytes from a 16 bit variable
struct packed_bit16
{
  uint8_t b0;
  uint8_t b1;
};

// union used for int16 and uint16 in combination with the above struct
union bit16
{
  int16_t i;
  uint16_t ui;
  packed_bit16 p;
};

// union used for int8 and uint8 in combination with the above struct, uint8_t is used to read single byte
// unbiased
union bit8
{
  int8_t i;
  uint8_t ui;
  uint8_t b0;
};
}  // namespace march
#endif  // MARCH_HARDWARE_ETHERCAT_PDO_TYPES_H
