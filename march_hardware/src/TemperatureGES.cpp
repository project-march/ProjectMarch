// Copyright 2019 Project March.
#include "march_hardware/EtherCAT/EthercatIO.h"

#include <march_hardware/TemperatureGES.h>

namespace march4cpp
{
TemperatureGES::TemperatureGES(int slaveIndex, int byteOffset) : Slave(slaveIndex)
{
  ROS_ASSERT_MSG(byteOffset >= 0, "Slave configuration error: byteOffset %d can not be negative.", byteOffset);
  this->byteOffset = byteOffset;
}

float TemperatureGES::getTemperature()
{
  // TODO(Martijn) read actual data from ethercat.
  union bit8 return_byte = get_input_bit8(static_cast<uint16>(slaveIndex), static_cast<uint8>(byteOffset));
  return static_cast<float>(return_byte.ui);
}

}  // namespace march4cpp
