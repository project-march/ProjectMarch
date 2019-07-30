// Copyright 2019 Project March.

#include <march_hardware/EtherCAT/EthercatIO.h>
#include <march_hardware/Encoder.h>
#include <cmath>
#include <ros/ros.h>

namespace march4cpp
{
Encoder::Encoder(int numberOfBits, int minPositionIU, int maxPositionIU, int zeroPositionIU, float safetyMarginRad)
{
  ROS_ASSERT_MSG(numberOfBits > 0 && numberOfBits <= 32, "Encoder resolution of %d is not within range (0, 32)",
                 numberOfBits);
  this->totalPositions = static_cast<int>(pow(2, numberOfBits) - 1);

  ROS_ASSERT_MSG(safetyMarginRad >= 0, "SafetyMarginRad %f is below zero", safetyMarginRad);

  this->safetyMarginRad = safetyMarginRad;
  this->slaveIndex = -1;
  this->positiveHardLimitIU = maxPositionIU;
  this->negativeHardLimitIU = minPositionIU;
  this->zeroPositionIU = zeroPositionIU;
  int safetyMarginIU = RadtoIU(safetyMarginRad) - this->zeroPositionIU;
  this->positiveSoftLimitIU = this->positiveHardLimitIU - safetyMarginIU;
  this->negativeSoftLimitIU = this->negativeHardLimitIU + safetyMarginIU;

  ROS_ASSERT_MSG(this->negativeSoftLimitIU < this->positiveSoftLimitIU,
                 "Invalid range of motion. Safety margin too large or "
                 "min/max position invalid. Minposition: %i IU, Maxposition: "
                 "%i IU, safetyMargin: %f rad",
                 this->negativeSoftLimitIU, this->positiveSoftLimitIU, this->safetyMarginRad);
}

float Encoder::getAngleRad(uint8_t ActualPositionByteOffset)
{
  return IUtoRad(getAngleIU(ActualPositionByteOffset));
}

int Encoder::getAngleIU(uint8_t ActualPositionByteOffset)
{
  if (this->slaveIndex == -1)
  {
    ROS_FATAL("Encoder has slaveIndex of -1");
  }
  union bit32 return_byte = get_input_bit32(this->slaveIndex, ActualPositionByteOffset);
  ROS_DEBUG("Encoder read (IU): %d", return_byte.i);
  return return_byte.i;
}

int Encoder::RadtoIU(float rad)
{
  return static_cast<int>(rad * totalPositions / (2 * M_PI) + zeroPositionIU);
}

float Encoder::IUtoRad(int iu)
{
  return static_cast<float>(iu - zeroPositionIU) * 2 * M_PI / totalPositions;
}

void Encoder::setSlaveIndex(int slaveIndex)
{
  this->slaveIndex = slaveIndex;
}

int Encoder::getSlaveIndex() const
{
  return this->slaveIndex;
}

bool Encoder::isWithinHardLimitsIU(int positionIU)
{
  return (positionIU > this->negativeHardLimitIU && positionIU < this->positiveHardLimitIU);
}

bool Encoder::isWithinSoftLimitsIU(int positionIU)
{
  return (positionIU > this->negativeSoftLimitIU && positionIU < this->positiveSoftLimitIU);
}

int Encoder::getPositiveSoftLimitIU() const
{
  return positiveSoftLimitIU;
}

int Encoder::getNegativeSoftLimitIU() const
{
  return negativeSoftLimitIU;
}

int Encoder::getPositiveHardLimitIU() const
{
  return positiveHardLimitIU;
}

int Encoder::getNegativeHardLimitIU() const
{
  return negativeHardLimitIU;
}

}  // namespace march4cpp
