// Copyright 2019 Project March.
#include "march_hardware/EtherCAT/EthercatSDO.h"
#include "ros/ros.h"

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
// TODO(Isha, Martijn, Tim) refactor this with more generic types
int sdo_bit8(int slave, uint32_t index, uint8_t sub, uint8_t value)
{
  ROS_DEBUG("sdo_bit8: slaveIndex %i, reg 0x%X, subindex %i, value 0x%X", slave, index, sub, value);
  int received_working_counter = ec_SDOwrite(slave, index, sub, FALSE, 1, &value, EC_TIMEOUTRXM);
  if (received_working_counter == 0)
  {
    ROS_FATAL("(sdo_bit8) Error occurred when writing, reg 0x%X, sub-index %i, data: 0x%X to slave %i", index, sub,
              value, slave);
  }
  return received_working_counter;
}

int sdo_bit16(int slave, uint32_t index, uint8_t sub, uint16_t value)
{
  ROS_DEBUG("sdo_bit16: slaveIndex %i, reg 0x%X, subindex %i, value 0x%X", slave, index, sub, value);
  int received_working_counter = ec_SDOwrite(slave, index, sub, FALSE, 2, &value, EC_TIMEOUTRXM);
  if (received_working_counter == 0)
  {
    ROS_FATAL("(sdo_bit16) Error occurred when writing, reg 0x%X, sub-index %i, data: 0x%X to slave %i", index, sub,
              value, slave);
  }
  return received_working_counter;
}

int sdo_bit32(int slave, uint32_t index, uint8_t sub, uint32_t value)
{
  ROS_DEBUG("sdo_bit32: slaveIndex %i, reg 0x%X, subindex %i, value 0x%X", slave, index, sub, value);
  int received_working_counter = ec_SDOwrite(slave, index, sub, FALSE, 4, &value, EC_TIMEOUTRXM);
  if (received_working_counter == 0)
  {
    ROS_FATAL("(sdo_bit32) Error occurred when writing, reg 0x%X, sub-index %i, data: 0x%X to slave %i", index, sub,
              value, slave);
  }
  return received_working_counter;
}
}  // namespace march4cpp
