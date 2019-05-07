// Copyright 2019 Project March.
#include "march_hardware/EtherCAT/EthercatIO.h"

#include <march_hardware/PowerDistributionBoard.h>

namespace march4cpp
{
PowerDistributionBoard::PowerDistributionBoard(int slaveIndex, int masterOkByteOffset,
                                               int powerDistributionBoardCurrentByteOffset)
  : Slave(slaveIndex)
{
  ROS_ASSERT_MSG(masterOkByteOffset >= 0, "Slave configuration error: temperatureByteOffset %d can not be negative.",
                 masterOkByteOffset);
  this->powerDistributionBoardCurrentByteOffset = powerDistributionBoardCurrentByteOffset;
  this->masterOkByteOffset = masterOkByteOffset;
}

float PowerDistributionBoard::getPowerDistributionBoardCurrent()
{
  union bit32 current = get_input_bit32(static_cast<uint16>(this->slaveIndex),
                                        static_cast<uint8>(this->powerDistributionBoardCurrentByteOffset));
  return current.f;
}

void PowerDistributionBoard::setMasterOk(bool isOk)
{
  bit8 isOkBit;
  isOkBit.ui = static_cast<uint8>(isOk);
  set_output_bit8(static_cast<uint16>(this->slaveIndex), static_cast<uint8>(this->masterOkByteOffset), isOkBit);
}

}  // namespace march4cpp
