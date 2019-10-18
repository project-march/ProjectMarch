// Copyright 2019 Project March.
#ifndef MARCH4CPP__ETHERCATIO_H
#define MARCH4CPP__ETHERCATIO_H

#include <stdint.h>
#include <stdio.h>

#include <soem/ethercattype.h>
#include <soem/nicdrv.h>
#include <soem/ethercatbase.h>
#include <soem/ethercatmain.h>
#include <soem/ethercatdc.h>
#include <soem/ethercatcoe.h>
#include <soem/ethercatfoe.h>
#include <soem/ethercatconfig.h>
#include <soem/ethercatprint.h>

namespace march4cpp
{
// struct used to easily get specific bytes from a 64 bit variable
typedef struct packed_bit64
{
  unsigned char b0;
  unsigned char b1;
  unsigned char b2;
  unsigned char b3;
  unsigned char b4;
  unsigned char b5;
  unsigned char b6;
  unsigned char b7;
}
packed_bit64;

// union used for int64 and uint64 in combination with the above struct
typedef union bit64
{
  int64 i;
  uint64 ui;
  packed_bit64 p;
}
bit64;

// struct used to send the x2 variable to the servo drive in the format that is required
typedef struct packed_sd_x2
{
  uint16 time;
  unsigned char reserved;  // empty but reserved space
  uint8 integrityCounter;  // represents a 7bit int where the last bit is also part of reserved
}
packed_sd_x2;

// struct used to easily get specific bytes from a 32 bit variable
typedef struct packed_bit32
{
  unsigned char b0;
  unsigned char b1;
  unsigned char b2;
  unsigned char b3;
}
packed_bit32;

// union used for int32, uint32, float and the x2 variable for the servo drive (sd) in combination with the above struct
typedef union bit32
{
  int32 i;
  uint32 ui;
  float f;
  packed_sd_x2 ppt;
  packed_bit32 p;
}
bit32;

// struct used to easily get specific bytes from a 16 bit variable
typedef struct packed_bit16
{
  unsigned char b0;
  unsigned char b1;
}
packed_bit16;

// union used for int16 and uint16 in combination with the above struct
typedef union bit16
{
  int16 i;
  uint16 ui;
  packed_bit16 p;
}
bit16;

// union used for int8 and uint8 in combination with the above struct, unsigned char is used to read single byte
// unbiased
typedef union bit8
{
  int8 i;
  uint8 ui;
  unsigned char b0;
}
bit8;

// functions to write #bit amounts of data to the EtherCAT train.
// Takes the slave number where 0 is the master and the first slave is 1.
// Takes a module index, which is the variable index for either the input or the output. (nth variable has index n)
union bit32 get_input_bit32(uint16 slave_no, uint8 module_index);
void set_output_bit32(uint16 slave_no, uint8 module_index, union bit32 value);
union bit32 get_output_bit32(uint16 slave_no, uint8 module_index);

union bit16 get_input_bit16(uint16 slave_no, uint8 module_index);
void set_output_bit16(uint16 slave_no, uint8 module_index, union bit16 value);

union bit8 get_input_bit8(uint16 slave_no, uint8 module_index);
void set_output_bit8(uint16 slave_no, uint8 module_index, union bit8 value);
union bit8 get_output_bit8(uint16 slave_no, uint8 module_index);

void set_output_bit(uint16 slave_no, uint8 module_index, uint8 value);

}  // namespace march4cpp
#endif  // MARCH4CPP__ETHERCAT_IO_
