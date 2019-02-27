//
// Created by projectmarch on 20-2-19.
//

#ifndef PROJECT_ENCODER_H
#define PROJECT_ENCODER_H

namespace march4cpp
{
class Encoder
{
private:
  int slaveIndex;
  int numberOfBytes;
  int minEncoderValue;
  int maxEncoderValue;
  float minDegvalue;
  float maxDegvalue;

public:
  Encoder() = default;

  Encoder(int numberOfBytes, int minEncoderValue, int maxEncoderValue, float minDegvalue,
          float maxDegValue);

  float getAngleRad();
  float getAngleDeg();

  int getAngleIU();

  float IUtoDeg(float iu);
  float IUtoRad(float iu);

  float DegtoIU(float deg);
  float RadtoIU(float rad);

  int getMinEncoderValue() const;
  int getMaxEncoderValue() const;
  float getMinDegvalue() const;
  float getMaxDegvalue() const;

  void setSlaveIndex(int slaveIndex);
  int getSlaveIndex();
};
}

#endif  // PROJECT_ENCODER_H
