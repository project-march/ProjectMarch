// Copyright 2019 Project March.
#include "march_hardware/EtherCAT/EthercatIO.h"

#include <march_hardware/PowerDistributionBoard.h>

namespace march4cpp
{
PowerDistributionBoard::PowerDistributionBoard(int slaveIndex, CurrentOffsets currentOffsets, StateOffsets stateOffsets)
  : Slave(slaveIndex)
{
  this->currentOffsets = currentOffsets;
  this->stateOffsets = stateOffsets;
}

float PowerDistributionBoard::getPowerDistributionBoardCurrent()
{
  union bit32 current = get_input_bit32(static_cast<uint16>(this->slaveIndex),
                                        static_cast<uint8>(this->currentOffsets.getPowerDistributionBoardByteOffset()));
  return current.f;
}

void PowerDistributionBoard::setMasterOk(bool isOk)
{
    bit8 isOkBit;
    isOkBit.ui = static_cast<uint8>(isOk);
    set_output_bit8(static_cast<uint16>(this->slaveIndex), static_cast<uint8>(this->stateOffsets.getMasterOkByteOffset()),
                    isOkBit);
}


void PowerDistributionBoard::setMasterShutDownAllowed(bool isAllowed)
{
    bit8 isAllowedBit;
    isAllowedBit.ui = static_cast<uint8>(isAllowed);
    set_output_bit8(static_cast<uint16>(this->slaveIndex), static_cast<uint8>(this->stateOffsets.getShutdownAllowedByteOffset()),
                    isAllowedBit);
}

float PowerDistributionBoard::getLowVoltageNet1Current() {
    union bit32 current = get_input_bit32(static_cast<uint16>(this->slaveIndex),
                                          static_cast<uint8>(this->currentOffsets.getLowVoltageNet1ByteOffset()));
    return current.f;
}

float PowerDistributionBoard::getLowVoltageNet2Current() {
    union bit32 current = get_input_bit32(static_cast<uint16>(this->slaveIndex),
                                          static_cast<uint8>(this->currentOffsets.getLowVoltageNet2ByteOffset()));
    return current.f;
}

float PowerDistributionBoard::getHighVoltageNetCurrent() {
    union bit32 current = get_input_bit32(static_cast<uint16>(this->slaveIndex),
                                          static_cast<uint8>(this->currentOffsets.getHighVoltageNetByteOffset()));
    return current.f;
}

bool PowerDistributionBoard::getMasterShutdownRequested() {
    union bit8 current = get_input_bit8(static_cast<uint16>(this->slaveIndex),
                                          static_cast<uint8>(this->stateOffsets.getShutdownByteOffset()));
    return current.ui;
}

}  // namespace march4cpp
