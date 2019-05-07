// Copyright 2019 Project March.
#ifndef MARCH_IV_CURRENTOFFSETS_H
#define MARCH_IV_CURRENTOFFSETS_H

#include <ostream>

class CurrentOffsets
{
  int powerDistributionBoardByteOffset;
  int LowVoltageNet1ByteOffset;
  int LowVoltageNet2ByteOffset;
  int HighVoltageNetByteOffset;

public:
  CurrentOffsets(int powerDistributionBoardByteOffset, int lowVoltageNet1ByteOffset, int lowVoltageNet2ByteOffset,
                 int highVoltageNetByteOffset)
    : powerDistributionBoardByteOffset(powerDistributionBoardByteOffset)
    , LowVoltageNet1ByteOffset(lowVoltageNet1ByteOffset)
    , LowVoltageNet2ByteOffset(lowVoltageNet2ByteOffset)
    , HighVoltageNetByteOffset(highVoltageNetByteOffset)
  {
  }

  CurrentOffsets()
  {
    powerDistributionBoardByteOffset = -1;
    LowVoltageNet1ByteOffset = -1;
    LowVoltageNet2ByteOffset = -1;
    HighVoltageNetByteOffset = -1;
  }

  int getPowerDistributionBoardByteOffset() const
  {
    return powerDistributionBoardByteOffset;
  }

  int getLowVoltageNet1ByteOffset() const
  {
    return LowVoltageNet1ByteOffset;
  }

  int getLowVoltageNet2ByteOffset() const
  {
    return LowVoltageNet2ByteOffset;
  }

  int getHighVoltageNetByteOffset() const
  {
    return HighVoltageNetByteOffset;
  }

  /** @brief Override comparison operator */
  friend bool operator==(const CurrentOffsets& lhs, const CurrentOffsets& rhs)
  {
    return lhs.powerDistributionBoardByteOffset == rhs.powerDistributionBoardByteOffset &&
           lhs.LowVoltageNet1ByteOffset == rhs.LowVoltageNet1ByteOffset &&
           lhs.LowVoltageNet2ByteOffset == rhs.LowVoltageNet2ByteOffset &&
           lhs.HighVoltageNetByteOffset == rhs.HighVoltageNetByteOffset;
  }
  /** @brief Override stream operator for clean printing */
  friend ::std::ostream& operator<<(std::ostream& os, const CurrentOffsets& currentOffsets)
  {
    return os << "powerDistributionBoardByteOffset: " << currentOffsets.powerDistributionBoardByteOffset << ", "
              << "LowVoltageNet1ByteOffset: " << currentOffsets.LowVoltageNet1ByteOffset << ", "
              << "LowVoltageNet2ByteOffset: " << currentOffsets.LowVoltageNet2ByteOffset << ", "
              << "HighVoltageNetByteOffset: " << currentOffsets.HighVoltageNetByteOffset;
  }
};

#endif  // MARCH_IV_CURRENTOFFSETS_H