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
  int minPositionIU;
  int maxPositionIU;
  int zeroPositionIU;
  float safetyMarginRad;

public:
  Encoder() = default;

  Encoder(int numberOfBits, int minPositionIU, int maxPositionIU, int zeroPositionIU, float safetyMarginRad);

  float getAngleRad();

  int getAngleIU();

  float IUtoRad(int iu);
  int RadtoIU(float rad);

  bool isValidTargetPositionIU(int targetPosIU);

  bool isValidPositionIU(int positionIU);
  void setSlaveIndex(int slaveIndex);

  int getSlaveIndex() const;
  int getMinPositionIU() const;
  int getMaxPositionIU() const;

  /** @brief Override comparison operator */
  friend bool operator==(const Encoder& lhs, const Encoder& rhs)
  {
    return lhs.slaveIndex == rhs.slaveIndex && lhs.totalPositions == rhs.totalPositions &&
           lhs.minPositionIU == rhs.minPositionIU && lhs.maxPositionIU == rhs.maxPositionIU &&
           lhs.zeroPositionIU == rhs.zeroPositionIU && lhs.safetyMarginRad == rhs.safetyMarginRad;
  }
  /** @brief Override stream operator for clean printing */
  friend ::std::ostream& operator<<(std::ostream& os, const Encoder& encoder)
  {
    return os << "Encoder: \n"
              << "slaveIndex: " << encoder.slaveIndex << "\n"
              << "totalPositions: " << encoder.totalPositions << "\n"
              << "minPositionIU: " << encoder.minPositionIU << "\n"
              << "maxPositionIU: " << encoder.maxPositionIU << "\n"
              << "zeroPositionIU: " << encoder.zeroPositionIU << "\n"
              << "safetyMarginRad: " << encoder.safetyMarginRad << "\n";
  }
};
}

#endif  // PROJECT_ENCODER_H
