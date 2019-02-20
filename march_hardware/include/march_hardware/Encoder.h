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
  int numberOfBytes;
  int minEncoderValue;
  int minDegvalue;

public:
  Encoder(){};

  Encoder(int numberOfBytes, int minEncoderValue, int minDegvalue);

  float getAngleRad();
  float getAngleDeg();

  float getAngle();
};
}

#endif  // PROJECT_ENCODER_H
