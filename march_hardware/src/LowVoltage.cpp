// Copyright 2019 Project March.
#include <march_hardware/LowVoltage.h>

namespace march4cpp
{
LowVoltage::LowVoltage(int slaveIndex, NetMonitorOffsets netMonitoringOffsets, NetDriverOffsets netDriverOffsets)
  : slaveIndex(slaveIndex), netMonitoringOffsets(netMonitoringOffsets), netDriverOffsets(netDriverOffsets)
{
}

LowVoltage::LowVoltage()
{
  slaveIndex = -1;
}

float LowVoltage::getNetCurrent(int netNumber)
{
  union bit32 current =
      get_input_bit32(static_cast<uint16>(this->slaveIndex),
                      static_cast<uint8>(this->netMonitoringOffsets.getLowVoltageNetCurrent(netNumber)));
  return current.f;
}

bool LowVoltage::getNetOperational(int netNumber)
{
  if (netNumber < 1 || netNumber > 2)
  {
    ROS_FATAL("Can't get operational state from low voltage net %d, there are only 2 low voltage nets", netNumber);
    throw std::exception();
  }
  union bit8 operational = get_input_bit8(static_cast<uint16>(this->slaveIndex),
                                          static_cast<uint8>(this->netMonitoringOffsets.getLowVoltageState()));
  // The last bit of the 8 bits represents net 1
  // The second to last bit of the 8 bits represents net 2
  return ((operational.ui >> (netNumber - 1)) & 1);
}

void LowVoltage::setNetOnOff(bool on, int netNumber)
{
  if (netNumber == 1)
  {
    ROS_FATAL("Can't turn low voltage net 1 on or off, would cause master suicide");
    throw std::exception();
  }
  else if (netNumber != 2)
  {
    ROS_FATAL("Can't turn low voltage net %d on or off, there are only 2 low voltage nets", netNumber);
    throw std::exception();
  }
  if (on && getNetOperational(netNumber))
  {
    ROS_WARN("Low voltage net %d is already on", netNumber);
    return;
  }
  else if (!on && !getNetOperational(netNumber))
  {
    ROS_WARN("Low voltage net %d is already off", netNumber);
    return;
  }

  uint8 currentStateLowVoltageNets = getNetsOperational();
  bit8 lowVoltageNets;
  lowVoltageNets.ui = 1 << (netNumber - 1);
  if (on)
  {
    lowVoltageNets.ui |= currentStateLowVoltageNets;
  }
  else
  {
    lowVoltageNets.ui = ~lowVoltageNets.ui;
    lowVoltageNets.ui &= currentStateLowVoltageNets;
  }
  ROS_INFO("low voltage nets: %d", lowVoltageNets.ui);
  ROS_INFO("offset: %d", this->netDriverOffsets.getLowVoltageNetOnOff());

  set_output_bit8(static_cast<uint16>(this->slaveIndex),
                  static_cast<uint8>(this->netDriverOffsets.getLowVoltageNetOnOff()), lowVoltageNets);
}

uint8 LowVoltage::getNetsOperational()
{
  union bit8 operational = get_input_bit8(static_cast<uint16>(this->slaveIndex),
                                          static_cast<uint8>(this->netMonitoringOffsets.getLowVoltageState()));
  return operational.ui;
}

}  // namespace march4cpp
