//
// Copyright 2019 Project March.
//

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

public:
  Encoder() = default;

  Encoder(int numberOfBits, int minPositionIU, int maxPositionIU, int zeroPositionIU);

  float getAngleRad();

  int getAngleIU();

  float IUtoRad(int iu);
  int RadtoIU(float rad);

  bool isValidTargetPositionIU(int targetPosIU);

  void setSlaveIndex(int slaveIndex);
  int getSlaveIndex() const;
};
}

#endif  // PROJECT_ENCODER_H
