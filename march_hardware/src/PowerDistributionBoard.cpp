// Copyright 2019 Project March.
#include "march_hardware/EtherCAT/EthercatIO.h"

#include <march_hardware/PowerDistributionBoard.h>

namespace march4cpp
{
PowerDistributionBoard::PowerDistributionBoard(int slaveIndex, NetMonitoringOffsets currentOffsets, StateOffsets stateOffsets)
  : Slave(slaveIndex)
{
  this->netMonitoringOffsets = currentOffsets;
  this->stateOffsets = stateOffsets;
}

float PowerDistributionBoard::getPowerDistributionBoardCurrent()
{
  union bit32 current = get_input_bit32(static_cast<uint16>(this->slaveIndex),
                                        static_cast<uint8>(this->netMonitoringOffsets.getPowerDistributionBoardCurrentByteOffset()));
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
                                          static_cast<uint8>(this->netMonitoringOffsets.getLowVoltageNet1CurrentByteOffset()));
    return current.f;
}

float PowerDistributionBoard::getLowVoltageNet2Current() {
    union bit32 current = get_input_bit32(static_cast<uint16>(this->slaveIndex),
                                          static_cast<uint8>(this->netMonitoringOffsets.getLowVoltageNet2CurrentByteOffset()));
    return current.f;
}

float PowerDistributionBoard::getHighVoltageNetCurrent() {
    union bit32 current = get_input_bit32(static_cast<uint16>(this->slaveIndex),
                                          static_cast<uint8>(this->netMonitoringOffsets.getHighVoltageNetCurrentByteOffset()));
    return current.f;
}

bool PowerDistributionBoard::getMasterShutdownRequested() {
    union bit8 current = get_input_bit8(static_cast<uint16>(this->slaveIndex),
                                          static_cast<uint8>(this->stateOffsets.getShutdownByteOffset()));
    return current.ui;
}

bool PowerDistributionBoard::getLowVoltageNet1Operational(){
    union bit8 operational = get_input_bit8(static_cast<uint16>(this->slaveIndex),
                                            static_cast<uint8>(this->netMonitoringOffsets.getLowVoltageStateByteOffset()));
    // The last bit of the 8 bits represents net 1
    return (operational.ui & 1);
}

bool PowerDistributionBoard::getLowVoltageNet2Operational(){
    union bit8 operational = get_input_bit8(static_cast<uint16>(this->slaveIndex),
                                            static_cast<uint8>(this->netMonitoringOffsets.getLowVoltageStateByteOffset()));
    // The second to last bit of the 8 bits represents net 2
    return ((operational.ui >> 1) & 1);
}

}  // namespace march4cpp
