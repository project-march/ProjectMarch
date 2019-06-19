// Copyright 2019 Project March.
#include <march_hardware/HighVoltage.h>

namespace march4cpp
{
HighVoltage::HighVoltage(int slaveIndex, NetMonitorOffsets netMonitoringOffsets, NetDriverOffsets netDriverOffsets)
  : slaveIndex(slaveIndex), netMonitoringOffsets(netMonitoringOffsets), netDriverOffsets(netDriverOffsets)
{
}

HighVoltage::HighVoltage()
{
  slaveIndex = -1;
}

float HighVoltage::getNetCurrent()
{
  union bit32 current = get_input_bit32(static_cast<uint16>(this->slaveIndex),
                                        static_cast<uint8>(this->netMonitoringOffsets.getHighVoltageNetCurrent()));
  return current.f;
}

bool HighVoltage::getNetOperational(int netNumber)
{
  if (netNumber < 1 || netNumber > 8)
  {
    ROS_ERROR_THROTTLE(2, "Can't get operational state from high voltage net %d, there are only 8 high voltage nets",
                       netNumber);
    throw std::invalid_argument("Only high voltage net 1 and 8 exist");
  }
  union bit8 operational = get_input_bit8(static_cast<uint16>(this->slaveIndex),
                                          static_cast<uint8>(this->netMonitoringOffsets.getHighVoltageState()));
  // The first bit of the 8 bits represents net 1 and so on till the last 8th bit which represents net 8.
  return ((operational.ui >> (netNumber - 1)) & 1);
}

bool HighVoltage::getOvercurrentTrigger(int netNumber)
{
  if (netNumber < 1 || netNumber > 8)
  {
    ROS_FATAL_THROTTLE(2, "Can't get overcurrent trigger from high voltage net %d, there are only 8 high voltage nets",
                       netNumber);
    throw std::exception();
  }
  union bit8 overcurrent =
      get_input_bit8(static_cast<uint16>(this->slaveIndex),
                     static_cast<uint8>(this->netMonitoringOffsets.getHighVoltageOvercurrentTrigger()));
  return ((overcurrent.ui >> (netNumber - 1)) & 1);
}

bool HighVoltage::getHighVoltageEnabled()
{
  union bit8 highVoltageEnabled = get_input_bit8(
      static_cast<uint16>(this->slaveIndex), static_cast<uint8>(this->netMonitoringOffsets.getHighVoltageEnabled()));
  return highVoltageEnabled.ui;
}

void HighVoltage::setNetOnOff(bool on, int netNumber)
{
  if (netNumber < 1 || netNumber > 8)
  {
    ROS_ERROR_THROTTLE(2, "Can't turn high voltage net %d on, there are only 8 high voltage nets", netNumber);
    throw std::invalid_argument("Only high voltage net 1 and 8 exist");
  }
  if (on && getNetOperational(netNumber))
  {
    ROS_WARN_THROTTLE(2, "High voltage net %d is already on", netNumber);
  }
  uint8 currentStateHighVoltageNets = getNetsOperational();
  bit8 highVoltageNets;
  highVoltageNets.ui = 1 << (netNumber - 1);
  if (on)
  {
    // Force bit of the respective net to one.
    highVoltageNets.ui |= currentStateHighVoltageNets;
  }
  else
  {
    // Force bit of the respective net to zero.
    highVoltageNets.ui = ~highVoltageNets.ui;
    highVoltageNets.ui &= currentStateHighVoltageNets;
  }
  set_output_bit8(static_cast<uint16>(this->slaveIndex),
                  static_cast<uint8>(this->netDriverOffsets.getHighVoltageNetOnOff()), highVoltageNets);
}

void HighVoltage::setHighVoltageOnOff(bool on)
{
  if (on && getHighVoltageEnabled())
  {
    ROS_ERROR_THROTTLE(2, "All-High-Voltage already enabled");
    throw std::runtime_error("All-High-Voltage already enabled");
  }
  else if (!on && !getHighVoltageEnabled())
  {
    ROS_ERROR_THROTTLE(2, "All-High-Voltage already disabled");
    throw std::runtime_error("All-High-Voltage already disabled");
  }
  if (on)
  {
    ROS_DEBUG_THROTTLE(2, "Trying to enable All-High-Voltage from software");
  }
  else
  {
    ROS_DEBUG_THROTTLE(2, "Trying to disable All-High-Voltage from software");
  }

  bit8 isOn;
  isOn.ui = static_cast<uint8>(on);
  set_output_bit8(static_cast<uint16>(this->slaveIndex),
                  static_cast<uint8>(this->netDriverOffsets.getAllHighVoltageOnOff()), isOn);
}

uint8 HighVoltage::getNetsOperational()
{
  union bit8 operational = get_input_bit8(static_cast<uint16>(this->slaveIndex),
                                          static_cast<uint8>(this->netMonitoringOffsets.getHighVoltageState()));
  return operational.ui;
}

}  // namespace march4cpp
