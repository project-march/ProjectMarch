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

  ROS_ASSERT_MSG(this->isValidPositionIU(minPositionIU), "MinPositionIU %d is not within range (0, %d)", minPositionIU,
                 this->totalPositions);
  ROS_ASSERT_MSG(this->isValidPositionIU(maxPositionIU), "MaxPositionIU %d is not within range (0, %d)", maxPositionIU,
                 this->totalPositions);
  ROS_ASSERT_MSG(this->isValidPositionIU(zeroPositionIU), "ZeroPositionIU %d is not within range (0, %d)",
                 zeroPositionIU, this->totalPositions);

  ROS_ASSERT_MSG(safetyMarginRad >= 0, "SafetyMarginRad %f is below zero", safetyMarginRad);

  this->safetyMarginRad = safetyMarginRad;
  this->slaveIndex = -1;

  this->zeroPositionIU = zeroPositionIU;

  int safetyMarginIU = RadtoIU(safetyMarginRad) - this->zeroPositionIU;

  this->minPositionIU = minPositionIU + safetyMarginIU;
  this->maxPositionIU = maxPositionIU - safetyMarginIU;

  ROS_ASSERT_MSG(this->minPositionIU < this->maxPositionIU, "Invalid range of motion. Safety margin too large or "
                                                            "min/max position invalid. Minposition: %i, Maxposition: "
                                                            "%i, safetyMargin: %f",
                 this->minPositionIU, this->maxPositionIU, this->safetyMarginRad);
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
  ROS_INFO("Encoder read (IU): %d", return_byte.i);
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

bool Encoder::isValidTargetPositionIU(int targetPosIU)
{
  return (targetPosIU > this->minPositionIU && targetPosIU < this->maxPositionIU);
}

int Encoder::getMinPositionIU() const
{
  return minPositionIU;
}

int Encoder::getMaxPositionIU() const
{
  return maxPositionIU;
}

bool Encoder::isValidPositionIU(int positionIU)
{
  return positionIU > 0 && positionIU <= this->totalPositions;
}

}  // namespace march4cpp
