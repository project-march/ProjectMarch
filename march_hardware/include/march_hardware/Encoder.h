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

  int positiveHardLimitIU;
  int negativeHardLimitIU;
  int positiveSoftLimitIU;
  int negativeSoftLimitIU;
  int zeroPositionIU;

  float safetyMarginRad;

public:
  Encoder() = default;

  Encoder(int numberOfBits, int minPositionIU, int maxPositionIU, int zeroPositionIU, float safetyMarginRad);

  float getAngleRad(uint8_t ActualPositionByteOffset);

  int getAngleIU(uint8_t ActualPositionByteOffset);

  float IUtoRad(int iu);
  int RadtoIU(float rad);

  bool isWithinHardLimits(int positionIU);
  bool isWithinSoftLimits(int positionIU);

  void setSlaveIndex(int slaveIndex);

  int getSlaveIndex() const;
  int getPositiveSoftLimit() const;
  int getNegativeSoftLimit() const;
  int getPositiveHardLimit() const;
  int getNegativeHardLimit() const;

  /** @brief Override comparison operator */
  friend bool operator==(const Encoder& lhs, const Encoder& rhs)
  {
    return lhs.slaveIndex == rhs.slaveIndex && lhs.totalPositions == rhs.totalPositions &&
           lhs.positiveSoftLimitIU == rhs.positiveSoftLimitIU && lhs.negativeSoftLimitIU == rhs.negativeSoftLimitIU &&
           lhs.positiveHardLimitIU == rhs.positiveHardLimitIU && lhs.negativeHardLimitIU == rhs.negativeHardLimitIU &&
           lhs.zeroPositionIU == rhs.zeroPositionIU && lhs.safetyMarginRad == rhs.safetyMarginRad;
  }
  /** @brief Override stream operator for clean printing */
  friend ::std::ostream& operator<<(std::ostream& os, const Encoder& encoder)
  {
    return os << "slaveIndex: " << encoder.slaveIndex << ", "
              << "totalPositions: " << encoder.totalPositions << ", "
              << "positiveHardLimit: " << encoder.positiveHardLimitIU << ", "
              << "negativeHardLimit: " << encoder.negativeHardLimitIU << ", "
              << "positiveSoftLimit: " << encoder.positiveSoftLimitIU << ", "
              << "negativeSoftLimit: " << encoder.negativeSoftLimitIU << ", "
              << "zeroPositionIU: " << encoder.zeroPositionIU << ", "
              << "safetyMarginRad: " << encoder.safetyMarginRad;
  }
};
}  // namespace march4cpp

#endif  // PROJECT_ENCODER_H
