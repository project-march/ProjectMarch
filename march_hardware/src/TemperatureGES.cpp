// Copyright 2019 Project March.
#include "march_hardware/EtherCAT/EthercatIO.h"

#include <march_hardware/TemperatureGES.h>

namespace march4cpp
{
TemperatureGES::TemperatureGES(int slaveIndex, int temperatureByteOffset) : Slave(slaveIndex)
{
  ROS_ASSERT_MSG(temperatureByteOffset >= 0, "Slave configuration error: temperatureByteOffset %d can not be negative.",
                 temperatureByteOffset);
  this->temperatureByteOffset = temperatureByteOffset;
}

float TemperatureGES::getTemperature()
{
  union bit32 temperature =
      get_input_bit32(static_cast<uint16>(slaveIndex), static_cast<uint8>(this->temperatureByteOffset));
  return temperature.f;
}

}  // namespace march4cpp
