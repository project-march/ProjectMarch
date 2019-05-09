// Copyright 2019 Project March.
#include "march_hardware/EtherCAT/EthercatIO.h"

#include <march_hardware/PowerDistributionBoard.h>

namespace march4cpp
{
PowerDistributionBoard::PowerDistributionBoard(int slaveIndex, NetMonitorOffsets netMonitoringOffsets,
                                               BootShutdownOffsets bootShutdownOffsets)
  : Slave(slaveIndex)
{
  this->netMonitoringOffsets = netMonitoringOffsets;
  this->bootShutdownOffsets = bootShutdownOffsets;
}

float PowerDistributionBoard::getPowerDistributionBoardCurrent()
{
  union bit32 current =
      get_input_bit32(static_cast<uint16>(this->slaveIndex),
                      static_cast<uint8>(this->netMonitoringOffsets.getPowerDistributionBoardCurrentByteOffset()));
  return current.f;
}

void PowerDistributionBoard::setMasterOk(bool isOk)
{
  bit8 isOkBit;
  isOkBit.ui = static_cast<uint8>(isOk);
  set_output_bit8(static_cast<uint16>(this->slaveIndex),
                  static_cast<uint8>(this->bootShutdownOffsets.getMasterOkByteOffset()), isOkBit);
}

void PowerDistributionBoard::setLowVoltageNetOnOff(bool on, int netNumber)
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
  if (on && getLowVoltageNetOperational(netNumber))
  {
    ROS_WARN("Low voltage net %d is already on", netNumber);
    return;
  }
  else if (!on && !getLowVoltageNetOperational(netNumber))
  {
    ROS_WARN("Low voltage net %d is already off", netNumber);
    return;
  }

  uint8 currentStateLowVoltageNets = getLowVoltageNetsOperational();
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
  set_output_bit8(static_cast<uint16>(this->slaveIndex),
                  static_cast<uint8>(this->netDriverOffsets.getLowVoltageNetOnOff()), lowVoltageNets);
}

void PowerDistributionBoard::setHighVoltageNetOnOff(bool on, int netNumber)
{
  if (netNumber < 1 || netNumber > 8)
  {
    ROS_FATAL("Can't turn high voltage net %d on, there are only 8 high voltage nets", netNumber);
    throw std::exception();
  }
  if (!on)
  {
    ROS_FATAL("You are not allowed to turn off high voltage nets this way, use the emergency switch");
    throw std::exception();
  }
  else if (getHighVoltageNetOperational(netNumber))
  {
    ROS_WARN("High voltage net %d is already on", netNumber);
  }
  bit8 isOn;
  isOn.ui = static_cast<uint8>(on);
  set_output_bit8(static_cast<uint16>(this->slaveIndex),
                  static_cast<uint8>(this->netDriverOffsets.getHighVoltageNetOnOff()), isOn);
}

void PowerDistributionBoard::setHighVoltageOvercurrentReset(bool on, int netNumber)
{
  bit8 isOn;
  isOn.ui = static_cast<uint8>(on);
  set_output_bit8(static_cast<uint16>(this->slaveIndex),
                  static_cast<uint8>(this->netDriverOffsets.getLowVoltageNetOnOff()), isOn);
}

void PowerDistributionBoard::setHighVoltageEmergencySwitchOnOff(bool on)
{
  bit8 isOn;
  isOn.ui = static_cast<uint8>(on);
  set_output_bit8(static_cast<uint16>(this->slaveIndex),
                  static_cast<uint8>(this->netDriverOffsets.getLowVoltageNetOnOff()), isOn);
}

void PowerDistributionBoard::setMasterShutDownAllowed(bool isAllowed)
{
  bit8 isAllowedBit;
  isAllowedBit.ui = static_cast<uint8>(isAllowed);
  set_output_bit8(static_cast<uint16>(this->slaveIndex),
                  static_cast<uint8>(this->bootShutdownOffsets.getShutdownAllowedByteOffset()), isAllowedBit);
}

float PowerDistributionBoard::getLowVoltageNetCurrent(int netNumber)
{
  union bit32 current =
      get_input_bit32(static_cast<uint16>(this->slaveIndex),
                      static_cast<uint8>(this->netMonitoringOffsets.getLowVoltageNetCurrentByteOffset(netNumber)));
  return current.f;
}

float PowerDistributionBoard::getHighVoltageNetCurrent()
{
  union bit32 current =
      get_input_bit32(static_cast<uint16>(this->slaveIndex),
                      static_cast<uint8>(this->netMonitoringOffsets.getHighVoltageNetCurrentByteOffset()));
  return current.f;
}

bool PowerDistributionBoard::getMasterShutdownRequested()
{
  union bit8 current = get_input_bit8(static_cast<uint16>(this->slaveIndex),
                                      static_cast<uint8>(this->bootShutdownOffsets.getShutdownByteOffset()));
  return current.ui;
}

bool PowerDistributionBoard::getEmergencyButtonTriggered()
{
  union bit8 emergencyButtonTriggered =
      get_input_bit8(static_cast<uint16>(this->slaveIndex),
                     static_cast<uint8>(this->netMonitoringOffsets.getEmergencyButtonTriggeredByteOffset()));
  return emergencyButtonTriggered.ui;
}

bool PowerDistributionBoard::getHighVoltageOvercurrentTrigger(int netNumber)
{
  if (netNumber < 1 || netNumber > 8)
  {
    ROS_FATAL("Can't get overcurrent trigger from high voltage net %d, there are only 8 high voltage nets", netNumber);
    throw std::exception();
  }
  union bit8 overcurrent =
      get_input_bit8(static_cast<uint16>(this->slaveIndex),
                     static_cast<uint8>(this->netMonitoringOffsets.getHighVoltageOvercurrentTriggerByteOffset()));
  return ((overcurrent.ui >> (netNumber - 1)) & 1);
}

bool PowerDistributionBoard::getLowVoltageNetOperational(int netNumber)
{
  if (netNumber < 1 || netNumber > 2)
  {
    ROS_FATAL("Can't get operational state from low voltage net %d, there are only 2 low voltage nets", netNumber);
    throw std::exception();
  }
  union bit8 operational =
      get_input_bit8(static_cast<uint16>(this->slaveIndex),
                     static_cast<uint8>(this->netMonitoringOffsets.getLowVoltageStateByteOffset()));
  // The last bit of the 8 bits represents net 1
  // The second to last bit of the 8 bits represents net 2
  return ((operational.ui >> (netNumber - 1)) & 1);
}

uint8 PowerDistributionBoard::getLowVoltageNetsOperational()
{
  union bit8 operational =
      get_input_bit8(static_cast<uint16>(this->slaveIndex),
                     static_cast<uint8>(this->netMonitoringOffsets.getLowVoltageStateByteOffset()));
  return operational.ui;
}

bool PowerDistributionBoard::getHighVoltageNetOperational(int netNumber)
{
  if (netNumber < 1 || netNumber > 8)
  {
    ROS_FATAL("Can't get operational state from high voltage net %d, there are only 8 high voltage nets", netNumber);
    throw std::exception();
  }
  union bit8 operational =
      get_input_bit8(static_cast<uint16>(this->slaveIndex),
                     static_cast<uint8>(this->netMonitoringOffsets.getHighVoltageStateByteOffset()));
  // The first bit of the 8 bits represents net 1 and so on till the last 8th bit which represents net 8.
  return ((operational.ui >> (netNumber - 1)) & 1);
}

uint8 PowerDistributionBoard::getHighVoltageNetsOperational()
{
  union bit8 operational =
      get_input_bit8(static_cast<uint16>(this->slaveIndex),
                     static_cast<uint8>(this->netMonitoringOffsets.getHighVoltageStateByteOffset()));
  return operational.ui;
}

}  // namespace march4cpp
