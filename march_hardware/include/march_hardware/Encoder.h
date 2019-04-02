// Copyright 2019 Project March.

#ifndef PROJECT_ENCODER_H
#define PROJECT_ENCODER_H

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

  float getAngleRad(uint8_t ActualPositionByteOffset);

  int getAngleIU(uint8_t ActualPositionByteOffset);

  float IUtoRad(int iu);
  int RadtoIU(float rad);

  bool isValidTargetPositionIU(int targetPosIU);

  void setSlaveIndex(int slaveIndex);
  int getSlaveIndex() const;

  int getMinPositionIU() const;
  int getMaxPositionIU() const;
};
}

#endif  // PROJECT_ENCODER_H
