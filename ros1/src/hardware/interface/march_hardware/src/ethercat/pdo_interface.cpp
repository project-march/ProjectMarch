// Copyright 2019 Project March.
#include "march_hardware/ethercat/pdo_interface.h"
#include "march_hardware/ethercat/pdo_types.h"

#include <cstdint>

#include <soem/ethercat.h>

namespace march {
void PdoInterfaceImpl::write8(
    uint16_t slave_index, uint8_t module_index, bit8 value)
{
    uint8_t* data_ptr = ec_slave[slave_index].outputs + module_index;

    *data_ptr = value.b0;
}

void PdoInterfaceImpl::write16(
    uint16_t slave_index, uint8_t module_index, bit16 value)
{
    uint8_t* data_ptr = ec_slave[slave_index].outputs + module_index;

    *(data_ptr++) = value.p.b0;
    *data_ptr = value.p.b1;
}

void PdoInterfaceImpl::write32(
    uint16_t slave_index, uint8_t module_index, bit32 value)
{
    uint8_t* data_ptr = ec_slave[slave_index].outputs + module_index;

    *(data_ptr++) = value.p.b0;
    *(data_ptr++) = value.p.b1;
    *(data_ptr++) = value.p.b2;
    *data_ptr = value.p.b3;
}

bit8 PdoInterfaceImpl::read8(uint16_t slave_index, uint8_t module_index) const
{
    const uint8_t* data_ptr = ec_slave[slave_index].inputs + module_index;

    bit8 return_value;
    return_value.b0 = *data_ptr;

    return return_value;
}

bit16 PdoInterfaceImpl::read16(uint16_t slave_index, uint8_t module_index) const
{
    const uint8_t* data_ptr = ec_slave[slave_index].inputs + module_index;

    bit16 result;
    result.p.b0 = *(data_ptr++);
    result.p.b1 = *data_ptr;

    return result;
}

bit32 PdoInterfaceImpl::read32(uint16_t slave_index, uint8_t module_index) const
{
    const uint8_t* data_ptr = ec_slave[slave_index].inputs + module_index;

    bit32 return_value;
    return_value.p.b0 = *(data_ptr++);
    return_value.p.b1 = *(data_ptr++);
    return_value.p.b2 = *(data_ptr++);
    return_value.p.b3 = *data_ptr;

    return return_value;
}
} // namespace march
