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
    ROS_FATAL("Can't get operational state from high voltage net %d, there are only 8 high voltage nets", netNumber);
    throw std::exception();
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
    ROS_FATAL("Can't get overcurrent trigger from high voltage net %d, there are only 8 high voltage nets", netNumber);
    throw std::exception();
  }
  union bit8 overcurrent =
      get_input_bit8(static_cast<uint16>(this->slaveIndex),
                     static_cast<uint8>(this->netMonitoringOffsets.getHighVoltageOvercurrentTrigger()));
  return ((overcurrent.ui >> (netNumber - 1)) & 1);
}

bool HighVoltage::getEmergencyButtonTrigger()
{
  union bit8 emergencyButtonTriggered =
      get_input_bit8(static_cast<uint16>(this->slaveIndex),
                     static_cast<uint8>(this->netMonitoringOffsets.getEmergencyButtonTriggered()));
  return emergencyButtonTriggered.ui;
}

void HighVoltage::setHighVoltageNetOnOff(bool on, int netNumber)
{
  if (netNumber < 1 || netNumber > 8)
  {
    ROS_FATAL("Can't turn high voltage net %d on, there are only 8 high voltage nets", netNumber);
    throw std::exception();
  }
  if (!on)
  {
    ROS_ERROR("You are not allowed to turn off high voltage nets this way, use the emergency switch");
    return;
  }
  else if (getNetOperational(netNumber))
  {
    ROS_WARN("High voltage net %d is already on", netNumber);
  }
  uint8 currentStateHighVoltageNets = getHighVoltageNetsOperational();
  bit8 highVoltageNets;
  highVoltageNets.ui = 1 << (netNumber - 1);
  if (on)
  {
    highVoltageNets.ui |= currentStateHighVoltageNets;
  }
  else
  {
    // This code is needed when this method is allowed to turn off high voltage
    highVoltageNets.ui = ~highVoltageNets.ui;
    highVoltageNets.ui &= currentStateHighVoltageNets;
  }
  set_output_bit8(static_cast<uint16>(this->slaveIndex),
                  static_cast<uint8>(this->netDriverOffsets.getHighVoltageNetOnOff()), highVoltageNets);
}

void HighVoltage::setHighVoltageEmergencySwitchOnOff(bool on)
{
  if (on && getEmergencyButtonTrigger())
  {
    ROS_WARN("Emergency switch already activated");
    return;
  }
  else if (!on && !getEmergencyButtonTrigger())
  {
    ROS_WARN("Emergency switch already deactivated");
    return;
  }
  if (on)
  {
    ROS_WARN("Emergency switch activated from software");
  }
  else
  {
    ROS_WARN("Emergency switch deactivated, high voltage on");
  }

  bit8 isOn;
  isOn.ui = static_cast<uint8>(on);
  set_output_bit8(static_cast<uint16>(this->slaveIndex),
                  static_cast<uint8>(this->netDriverOffsets.getHighVoltageEmergencySwitchOnOff()), isOn);
}

uint8 HighVoltage::getHighVoltageNetsOperational()
{
  union bit8 operational = get_input_bit8(static_cast<uint16>(this->slaveIndex),
                                          static_cast<uint8>(this->netMonitoringOffsets.getHighVoltageState()));
  return operational.ui;
}


}  // namespace march4cpp
