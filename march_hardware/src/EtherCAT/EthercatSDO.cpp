#include "march_hardware/EtherCAT/EthercatSDO.h"

extern "C" {
#include "ethercat.h"
}

int sdo_bit8(int slave, uint32_t index, uint8_t sub, uint8_t value)
{
  return ec_SDOwrite(slave, index, sub, FALSE, 1, &value, EC_TIMEOUTRXM);
}

int sdo_bit16(int slave, uint32_t index, uint8_t sub, uint16_t value)
{
  return ec_SDOwrite(slave, index, sub, FALSE, 2, &value, EC_TIMEOUTRXM);
}

int sdo_bit32(int slave, uint32_t index, uint8_t sub, uint32_t value)
{
  return ec_SDOwrite(slave, index, sub, FALSE, 4, &value, EC_TIMEOUTRXM);
}
