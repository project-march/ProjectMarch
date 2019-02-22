//
// Created by projectmarch on 20-2-19.
//

#include <march_hardware/EtherCAT/EthercatIO.h>
#include <march_hardware/Encoder.h>
#include <cmath>
#include <ros/ros.h>

namespace march4cpp
{
Encoder::Encoder(int numberOfBytes, int minEncoderValue, int maxEncoderValue, float minDegValue, float maxDegValue)
{
  this->numberOfBytes = numberOfBytes;
  this->minEncoderValue = minEncoderValue;
  this->maxEncoderValue = maxEncoderValue;
  this->minDegvalue = minDegValue;
  this->maxDegvalue = maxDegValue;
}

float Encoder::getAngleRad()
{
  return IUtoRad(getAngleIU());
}

// TODO refactor to doubles;
float Encoder::getAngleDeg()
{
  return IUtoDeg(getAngleIU());
}

int Encoder::getAngleIU()
{
  // TODO(Martijn) read absolute position instead of motor position when test joint is fixed
  union bit32 return_byte = get_input_bit32(2, 2);
  return return_byte.i;
}

float Encoder::IUtoDeg(float iu)
{
  auto bits = static_cast<float>(pow(2, this->numberOfBytes));
  return (iu - this->minEncoderValue) / bits * 360 + this->minDegvalue;
}

float Encoder::IUtoRad(float iu)
{
  auto bits = static_cast<float>(pow(2, this->numberOfBytes));
  return static_cast<float>((iu - this->minEncoderValue) / bits * 2 * M_PI + this->minDegvalue);
}

int Encoder::getMinEncoderValue() const
{
  return minEncoderValue;
}

int Encoder::getMaxEncoderValue() const
{
  return maxEncoderValue;
}

float Encoder::getMinDegvalue() const
{
  return minDegvalue;
}

float Encoder::getMaxDegvalue() const
{
  return maxDegvalue;
}
}