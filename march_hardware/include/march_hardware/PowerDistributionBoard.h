// Copyright 2019 Project March.
#ifndef MARCH4CPP__POWERDISTRIBUTIONBOARD_H
#define MARCH4CPP__POWERDISTRIBUTIONBOARD_H

#include <stdint.h>
#include <march_hardware/Slave.h>

namespace march4cpp
{
class PowerDistributionBoard : public Slave
{
private:
  int masterOkByteOffset;
  int powerDistributionBoardCurrentByteOffset;

public:
  PowerDistributionBoard(int slaveIndex, int masterOkByteOffset, int powerDistributionBoardCurrentByteOffset);

  PowerDistributionBoard()
  {
    masterOkByteOffset = -1;
    slaveIndex = -1;
  };

  float getPowerDistributionBoardCurrent();

  void setMasterOk(bool isOk);

  /** @brief Override comparison operator */
  friend bool operator==(const PowerDistributionBoard& lhs, const PowerDistributionBoard& rhs)
  {
    return lhs.slaveIndex == rhs.slaveIndex && lhs.masterOkByteOffset == rhs.masterOkByteOffset &&
           lhs.powerDistributionBoardCurrentByteOffset == rhs.powerDistributionBoardCurrentByteOffset;
  }
  /** @brief Override stream operator for clean printing */
  friend ::std::ostream& operator<<(std::ostream& os, const PowerDistributionBoard& powerDistributionBoard)
  {
    return os << "slaveIndex: " << powerDistributionBoard.slaveIndex << ", "
              << "powerDistributionBoardCurrentByteOffset: "
              << powerDistributionBoard.powerDistributionBoardCurrentByteOffset << ", "
              << "masterOkByteOffset: "
              << powerDistributionBoard.masterOkByteOffset;
  }
};
}  // namespace march4cpp
#endif
