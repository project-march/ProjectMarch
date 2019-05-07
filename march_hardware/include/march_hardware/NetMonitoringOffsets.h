// Copyright 2019 Project March.
#ifndef MARCH_IV_NETMONITORINGOFFSETS_H
#define MARCH_IV_NETMONITORINGOFFSETS_H

#include <ostream>

class NetMonitoringOffsets
{
  int powerDistributionBoardCurrentByteOffset;
  int LowVoltageNet1CurrentByteOffset;
  int LowVoltageNet2CurrentByteOffset;
  int HighVoltageNetCurrentByteOffset;
  int LowVoltageStateByteOffset;

public:
  NetMonitoringOffsets(int powerDistributionBoardCurrentByteOffset, int lowVoltageNet1CurrentByteOffset, int lowVoltageNet2CurrentByteOffset,
                 int highVoltageNetCurrentByteOffset, int LowVoltageStateByteOffset)
    : powerDistributionBoardCurrentByteOffset(powerDistributionBoardCurrentByteOffset)
    , LowVoltageNet1CurrentByteOffset(lowVoltageNet1CurrentByteOffset)
    , LowVoltageNet2CurrentByteOffset(lowVoltageNet2CurrentByteOffset)
    , HighVoltageNetCurrentByteOffset(highVoltageNetCurrentByteOffset)
    , LowVoltageStateByteOffset(LowVoltageStateByteOffset)
  {
  }

  NetMonitoringOffsets()
  {
    powerDistributionBoardCurrentByteOffset = -1;
    LowVoltageNet1CurrentByteOffset = -1;
    LowVoltageNet2CurrentByteOffset = -1;
    HighVoltageNetCurrentByteOffset = -1;
  }

    int getPowerDistributionBoardCurrentByteOffset() const {
        return powerDistributionBoardCurrentByteOffset;
    }

    int getLowVoltageNet1CurrentByteOffset() const {
        return LowVoltageNet1CurrentByteOffset;
    }

    int getLowVoltageNet2CurrentByteOffset() const {
        return LowVoltageNet2CurrentByteOffset;
    }

    int getHighVoltageNetCurrentByteOffset() const {
        return HighVoltageNetCurrentByteOffset;
    }

    int getLowVoltageStateByteOffset() const {
        return LowVoltageStateByteOffset;
    }

    /** @brief Override comparison operator */
  friend bool operator==(const NetMonitoringOffsets& lhs, const NetMonitoringOffsets& rhs)
  {
    return lhs.powerDistributionBoardCurrentByteOffset == rhs.powerDistributionBoardCurrentByteOffset &&
           lhs.LowVoltageNet1CurrentByteOffset == rhs.LowVoltageNet1CurrentByteOffset &&
           lhs.LowVoltageNet2CurrentByteOffset == rhs.LowVoltageNet2CurrentByteOffset &&
           lhs.HighVoltageNetCurrentByteOffset == rhs.HighVoltageNetCurrentByteOffset;
  }
  /** @brief Override stream operator for clean printing */
  friend ::std::ostream& operator<<(std::ostream& os, const NetMonitoringOffsets& currentOffsets)
  {
    return os << "powerDistributionBoardCurrentByteOffset: " << currentOffsets.powerDistributionBoardCurrentByteOffset << ", "
              << "LowVoltageNet1CurrentByteOffset: " << currentOffsets.LowVoltageNet1CurrentByteOffset << ", "
              << "LowVoltageNet2CurrentByteOffset: " << currentOffsets.LowVoltageNet2CurrentByteOffset << ", "
              << "HighVoltageNetCurrentByteOffset: " << currentOffsets.HighVoltageNetCurrentByteOffset;
  }
};

#endif  //MARCH_IV_NETMONITORINGOFFSETS_H