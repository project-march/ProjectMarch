// Copyright 2019 Project March.

#include <march_hardware/PowerDistributionBoard.h>

namespace march
{
PowerDistributionBoard::PowerDistributionBoard(int slaveIndex, NetMonitorOffsets netMonitoringOffsets,
                                               NetDriverOffsets netDriverOffsets,
                                               BootShutdownOffsets bootShutdownOffsets)
  : Slave(slaveIndex)
{
  this->netMonitoringOffsets = netMonitoringOffsets;
  this->bootShutdownOffsets = bootShutdownOffsets;
  this->netDriverOffsets = netDriverOffsets;
  this->highVoltage = HighVoltage(slaveIndex, netMonitoringOffsets, netDriverOffsets);
  this->lowVoltage = LowVoltage(slaveIndex, netMonitoringOffsets, netDriverOffsets);
  this->masterOnlineToggle = false;
}

float PowerDistributionBoard::getPowerDistributionBoardCurrent()
{
  union bit32 current =
      get_input_bit32(static_cast<uint16>(this->slaveIndex),
                      static_cast<uint8>(this->netMonitoringOffsets.getPowerDistributionBoardCurrent()));
  return current.f;
}

void PowerDistributionBoard::setMasterOnline()
{
  bit8 isOkBit;
  // By continuously flipping the master online toggle we let the pdb know we are still connected.
  this->masterOnlineToggle = !this->masterOnlineToggle;
  isOkBit.ui = static_cast<uint8>(this->masterOnlineToggle);
  set_output_bit8(static_cast<uint16>(this->slaveIndex),
                  static_cast<uint8>(this->bootShutdownOffsets.getMasterOkByteOffset()), isOkBit);
}

void PowerDistributionBoard::setMasterShutDownAllowed(bool isAllowed)
{
  bit8 isAllowedBit;
  isAllowedBit.ui = static_cast<uint8>(isAllowed);
  set_output_bit8(static_cast<uint16>(this->slaveIndex),
                  static_cast<uint8>(this->bootShutdownOffsets.getShutdownAllowedByteOffset()), isAllowedBit);
}

bool PowerDistributionBoard::getMasterShutdownRequested()
{
  union bit8 masterShutdownRequestedBit = get_input_bit8(
      static_cast<uint16>(this->slaveIndex), static_cast<uint8>(this->bootShutdownOffsets.getShutdownByteOffset()));
  return masterShutdownRequestedBit.ui;
}

HighVoltage PowerDistributionBoard::getHighVoltage()
{
  return highVoltage;
}

LowVoltage PowerDistributionBoard::getLowVoltage()
{
  return lowVoltage;
}

}  // namespace march
