// Copyright 2019 Project March.
#include "march_hardware/power/power_distribution_board.h"
#include "march_hardware/communication/ethercat/pdo_types.h"

namespace march
{
PowerDistributionBoard::PowerDistributionBoard(const Slave& slave, NetMonitorOffsets netMonitoringOffsets,
                                               NetDriverOffsets netDriverOffsets,
                                               BootShutdownOffsets bootShutdownOffsets)
  : Slave(slave)
  , netMonitoringOffsets(netMonitoringOffsets)
  , netDriverOffsets(netDriverOffsets)
  , bootShutdownOffsets(bootShutdownOffsets)
  , highVoltage(*this, netMonitoringOffsets, netDriverOffsets)
  , lowVoltage(*this, netMonitoringOffsets, netDriverOffsets)
  , masterOnlineToggle(false)
{
}

float PowerDistributionBoard::getPowerDistributionBoardCurrent()
{
  bit32 current = this->read32(this->netMonitoringOffsets.getPowerDistributionBoardCurrent());
  return current.f;
}

void PowerDistributionBoard::setMasterOnline()
{
  bit8 isOkBit;
  // By continuously flipping the master online toggle we let the pdb know we are still connected.
  this->masterOnlineToggle = !this->masterOnlineToggle;
  isOkBit.ui = this->masterOnlineToggle;
  this->write8(this->bootShutdownOffsets.getMasterOkByteOffset(), isOkBit);
}

void PowerDistributionBoard::setMasterShutDownAllowed(bool isAllowed)
{
  bit8 isAllowedBit;
  isAllowedBit.ui = isAllowed;
  this->write8(this->bootShutdownOffsets.getShutdownAllowedByteOffset(), isAllowedBit);
}

bool PowerDistributionBoard::getMasterShutdownRequested()
{
  bit8 masterShutdownRequestedBit = this->read8(this->bootShutdownOffsets.getShutdownByteOffset());
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
