// Copyright 2019 Project March.
#include "march_hardware/EtherCAT/EthercatIO.h"

namespace march
{
union bit64 get_input_bit64(uint16_t slave_no, uint8_t module_index)
{
  union bit64 return_value;
  uint8_t* data_ptr;
  /* Get the IO map pointer from the ec_slave struct */
  data_ptr = ec_slave[slave_no].inputs;
  /* Move pointer to correct module index */
  data_ptr += module_index * 8;  // Probably wrong!
                                 /* Read value byte by byte since all targets can't handle misaligned
                                  * addresses
                                  */
  return_value.p.b0 = *data_ptr++;
  return_value.p.b1 = *data_ptr++;
  return_value.p.b2 = *data_ptr++;
  return_value.p.b3 = *data_ptr++;
  return_value.p.b4 = *data_ptr++;
  return_value.p.b5 = *data_ptr++;
  return_value.p.b6 = *data_ptr++;
  return_value.p.b7 = *data_ptr;

  return return_value;
}

void set_output_bit64(uint16_t slave_no, uint8_t module_index, union bit64 value)
{
  uint8_t* data_ptr;
  data_ptr = ec_slave[slave_no].outputs;
  data_ptr += module_index * 8;  // Probably wrong!

  *data_ptr++ = value.p.b0;
  *data_ptr++ = value.p.b1;
  *data_ptr++ = value.p.b2;
  *data_ptr++ = value.p.b3;
  *data_ptr++ = value.p.b4;
  *data_ptr++ = value.p.b5;
  *data_ptr++ = value.p.b6;
  *data_ptr = value.p.b7;
}

union bit32 get_input_bit32(uint16_t slave_no, uint8_t module_index)
{
  union bit32 return_value;
  uint8_t* data_ptr;
  data_ptr = ec_slave[slave_no].inputs;
  data_ptr += module_index;

  return_value.p.b0 = *data_ptr++;
  return_value.p.b1 = *data_ptr++;
  return_value.p.b2 = *data_ptr++;
  return_value.p.b3 = *data_ptr;

  return return_value;
}

void set_output_bit32(uint16_t slave_no, uint8_t module_index, union bit32 value)
{
  uint8_t* data_ptr;
  data_ptr = ec_slave[slave_no].outputs;
  data_ptr += module_index;

  *data_ptr++ = value.p.b0;
  *data_ptr++ = value.p.b1;
  *data_ptr++ = value.p.b2;
  *data_ptr = value.p.b3;
}

union bit32 get_output_bit32(uint16_t slave_no, uint8_t module_index)
{
  union bit32 return_value;
  uint8_t* data_ptr;
  data_ptr = ec_slave[slave_no].outputs;
  data_ptr += module_index;

  return_value.p.b0 = *data_ptr++;
  return_value.p.b1 = *data_ptr++;
  return_value.p.b2 = *data_ptr++;
  return_value.p.b3 = *data_ptr;

  return return_value;
}

union bit16 get_input_bit16(uint16_t slave_no, uint8_t module_index)
{
  union bit16 return_value;
  uint8_t* data_ptr;
  data_ptr = ec_slave[slave_no].inputs;
  data_ptr += module_index;

  return_value.p.b0 = *data_ptr++;
  return_value.p.b1 = *data_ptr;

  return return_value;
}

void set_output_bit16(uint16_t slave_no, uint8_t module_index, union bit16 value)
{
  uint8_t* data_ptr;
  data_ptr = ec_slave[slave_no].outputs;
  data_ptr += module_index;

  *data_ptr++ = value.p.b0;
  *data_ptr = value.p.b1;
}

union bit8 get_input_bit8(uint16_t slave_no, uint8_t module_index)
{
  union bit8 return_value;
  uint8_t* data_ptr;
  data_ptr = ec_slave[slave_no].inputs;
  data_ptr += module_index;

  return_value.b0 = *data_ptr;

  return return_value;
}

void set_output_bit8(uint16_t slave_no, uint8_t module_index, union bit8 value)
{
  uint8_t* data_ptr;
  data_ptr = ec_slave[slave_no].outputs;
  data_ptr += module_index;

  *data_ptr = value.b0;
}

union bit8 get_output_bit8(uint16_t slave_no, uint8_t module_index)
{
  union bit8 return_value;
  uint8_t* data_ptr;
  data_ptr = ec_slave[slave_no].outputs;
  data_ptr += module_index;

  return_value.b0 = *data_ptr;

  return return_value;
}

void set_output_bit(uint16_t slave_no, uint8_t module_index, uint8_t value)
{
  /* Get the the startbit position in slaves IO byte */
  uint8_t start_bit = ec_slave[slave_no].Ostartbit;
  /* Set or Clear bit */
  if (value == 0)
  {
    *ec_slave[slave_no].outputs &= ~(1u << (module_index - 1u + start_bit));
  }
  else
  {
    *ec_slave[slave_no].outputs |= (1u << (module_index - 1u + start_bit));
  }
}

}  // namespace march
