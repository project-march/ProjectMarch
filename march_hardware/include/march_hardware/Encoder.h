//
// Created by projectmarch on 20-2-19.
//

#ifndef PROJECT_ENCODER_H
#define PROJECT_ENCODER_H

#include "AngleEncoder.h"
namespace march4cpp
{
class Encoder : public march4cpp::AngleEncoder
{
private:
  int numberOfBytes;
  int minEncoderValue;
  int minDegvalue;

public:
  Encoder(){};

  Encoder(int numberOfBytes, int minEncoderValue, int minDegvalue);

  float getAngleRad() override;
  float getAngleDeg() override;
  float getAngle() override;
};
}

#endif  // PROJECT_ENCODER_H
