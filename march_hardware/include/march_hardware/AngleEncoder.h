// Copyright 2019 Project March.
#ifndef PROJECT_SIMPLEENCODER_H
#define PROJECT_SIMPLEENCODER_H

namespace march4cpp
{
class AngleEncoder
{
public:
  virtual float getAngleRad() = 0;
  virtual float getAngleDeg() = 0;
  virtual float getAngle() = 0;
};
}

#endif  // PROJECT_SIMPLEENCODER_H
