// Copyright 2019 Project March.

#ifndef MARCH_HARDWARE_ENCODER_H
#define MARCH_HARDWARE_ENCODER_H

#include <ostream>

namespace march
{
class Encoder
{
private:
  int slaveIndex;
  int totalPositions;

  int32_t upperHardLimitIU;
  int32_t lowerHardLimitIU;
  int32_t upperSoftLimitIU;
  int32_t lowerSoftLimitIU;
  int32_t zeroPositionIU;

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
  Encoder(int numberOfBits, int32_t minPositionIU, int32_t maxPositionIU, int32_t zeroPositionIU,
          float safetyMarginRad);

  double getAngleRad(uint8_t ActualPositionByteOffset);

  int32_t getAngleIU(uint8_t ActualPositionByteOffset);

  double IUtoRad(int32_t iu);
  int32_t RadtoIU(double rad);

  bool isWithinHardLimitsIU(int32_t positionIU);
  bool isWithinSoftLimitsIU(int32_t positionIU);
  bool isValidTargetIU(int32_t currentIU, int32_t targetIU);

  void setSlaveIndex(int slaveIndex);

  int getSlaveIndex() const;
  int32_t getUpperSoftLimitIU() const;
  int32_t getLowerSoftLimitIU() const;
  int32_t getUpperHardLimitIU() const;
  int32_t getLowerHardLimitIU() const;

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
}  // namespace march

#endif  // MARCH_HARDWARE_ENCODER_H
