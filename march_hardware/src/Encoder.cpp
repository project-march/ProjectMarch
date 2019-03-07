//
// Copyright 2019 Project March.
//

#include <march_hardware/EtherCAT/EthercatIO.h>
#include <march_hardware/Encoder.h>
#include <cmath>
#include <ros/ros.h>

namespace march4cpp
{
Encoder::Encoder(int numberOfBits, int minPositionIU, int maxPositionIU, int zeroPositionIU)
{
  this->slaveIndex = -1;
  this->totalPositions = static_cast<int>(pow(2, numberOfBits));
  this->minPositionIU = minPositionIU;
  this->maxPositionIU = maxPositionIU;
  this->zeroPositionIU = zeroPositionIU;
}

float Encoder::getAngleRad()
{
  return IUtoRad(getAngleIU());
}

int Encoder::getAngleIU()
{
  if (this->slaveIndex == -1)
  {
    ROS_FATAL("Encoder has slaveIndex of -1");
  }
  union bit32 return_byte = get_input_bit32(this->slaveIndex, 2);
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

}  // namespace march4cpp
