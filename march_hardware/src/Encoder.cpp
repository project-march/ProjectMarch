//
// Created by projectmarch on 20-2-19.
//

#include <march_hardware/EtherCAT/EthercatIO.h>
#include <march_hardware/Encoder.h>
#include <cmath>
#include <ros/ros.h>

namespace march4cpp
{
Encoder::Encoder(int numberOfBytes, int minEncoderValue, int minDegvalue)
{
  this->numberOfBytes = numberOfBytes;
  this->minEncoderValue = minEncoderValue;
  this->minDegvalue = minDegvalue;
}


// TODO refactor to doubles;
float Encoder::getAngleDeg()
{
    float bits = static_cast<float>(pow(2, 16));
    float angle = getAngle();
    return (angle - this->minEncoderValue)/bits*360 + this->minDegvalue;
}

float Encoder::getAngleRad(){
    return getAngleDeg()*M_PI/180;
}

float Encoder::getAngle()
{
  // TODO(Martijn) read absolute position instead of motor position when test joint is fixed
  union bit32 return_byte = get_input_bit32(2, 2);
  return (float)return_byte.i;
}
}