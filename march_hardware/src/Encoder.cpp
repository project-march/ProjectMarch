// Copyright 2019 Project March.

#include <march_hardware/EtherCAT/EthercatIO.h>
#include <march_hardware/Encoder.h>
#include <cmath>
#include <ros/ros.h>

namespace march4cpp
{
Encoder::Encoder(int numberOfBits, int minPositionIU, int maxPositionIU, int zeroPositionIU, float safetyMarginRad)
{
  this->safetyMarginRad = safetyMarginRad;
  this->slaveIndex = -1;
  this->totalPositions = static_cast<int>(pow(2, numberOfBits));
  this->zeroPositionIU = zeroPositionIU;

  int safetyMarginIU = RadtoIU(safetyMarginRad) - this->zeroPositionIU;

  this->minPositionIU = minPositionIU + safetyMarginIU;
  this->maxPositionIU = maxPositionIU - safetyMarginIU;

  if (this->minPositionIU >= this->maxPositionIU)
  {
    ROS_FATAL("No valid range of motion. Safety margin too large or min/max position invalid. Minposition: %i, "
              "Maxposition: %i, safetyMargin: %f",
              this->minPositionIU, this->maxPositionIU, this->safetyMarginRad);
    throw std::invalid_argument("No valid range of motion. Safety margin too large or min/max position invalid.");
  }
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

}  // namespace march4cpp
