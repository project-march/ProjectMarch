// Copyright 2019 Project March.

#ifndef PROJECT_ENCODER_H
#define PROJECT_ENCODER_H

#include <ostream>

namespace march4cpp
{
class Encoder
{
private:
  int slaveIndex;
  int totalPositions;

  int upperHardLimitIU;
  int lowerHardLimitIU;
  int upperSoftLimitIU;
  int lowerSoftLimitIU;
  int zeroPositionIU;

  float safetyMarginRad;

public:
  Encoder()
    : slaveIndex(-1)
    , totalPositions(0)
    , upperHardLimitIU(0)
    , lowerHardLimitIU(0)
    , upperSoftLimitIU(0)
    , lowerSoftLimitIU(0)
    , zeroPositionIU(0)
    , safetyMarginRad(0)
  {
  }
  Encoder(int numberOfBits, int minPositionIU, int maxPositionIU, int zeroPositionIU, float safetyMarginRad);

  float getAngleRad(uint8_t ActualPositionByteOffset);

  int getAngleIU(uint8_t ActualPositionByteOffset);

  float IUtoRad(int iu);
  int RadtoIU(float rad);

  bool isWithinHardLimitsIU(int positionIU);
  bool isWithinSoftLimitsIU(int positionIU);

  void setSlaveIndex(int slaveIndex);

  int getSlaveIndex() const;
  int getUpperSoftLimitIU() const;
  int getLowerSoftLimitIU() const;
  int getUpperHardLimitIU() const;
  int getLowerHardLimitIU() const;

  /** @brief Override comparison operator */
  friend bool operator==(const Encoder& lhs, const Encoder& rhs)
  {
    return lhs.slaveIndex == rhs.slaveIndex && lhs.totalPositions == rhs.totalPositions &&
           lhs.upperSoftLimitIU == rhs.upperSoftLimitIU && lhs.lowerSoftLimitIU == rhs.lowerSoftLimitIU &&
           lhs.upperHardLimitIU == rhs.upperHardLimitIU && lhs.lowerHardLimitIU == rhs.lowerHardLimitIU &&
           lhs.zeroPositionIU == rhs.zeroPositionIU && lhs.safetyMarginRad == rhs.safetyMarginRad;
  }
  /** @brief Override stream operator for clean printing */
  friend ::std::ostream& operator<<(std::ostream& os, const Encoder& encoder)
  {
    return os << "slaveIndex: " << encoder.slaveIndex << ", "
              << "totalPositions: " << encoder.totalPositions << ", "
              << "upperHardLimit: " << encoder.upperHardLimitIU << ", "
              << "lowerHardLimit: " << encoder.lowerHardLimitIU << ", "
              << "upperSoftLimit: " << encoder.upperSoftLimitIU << ", "
              << "lowerSoftLimit: " << encoder.lowerSoftLimitIU << ", "
              << "zeroPositionIU: " << encoder.zeroPositionIU << ", "
              << "safetyMarginRad: " << encoder.safetyMarginRad;
  }
};
}  // namespace march4cpp

#endif  // PROJECT_ENCODER_H
