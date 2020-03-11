// Copyright 2019 Project March.
#include "march_hardware/encoder/Encoder.h"
#include "march_hardware/EtherCAT/EthercatIO.h"
#include "march_hardware/error/hardware_exception.h"

#include <ros/ros.h>

namespace march
{
Encoder::Encoder(size_t number_of_bits) : total_positions_(Encoder::calculateTotalPositions(number_of_bits))
{
}

int32_t Encoder::getAngleIU(uint8_t byte_offset) const
{
  if (this->slave_index_ == -1)
  {
    ROS_FATAL("Encoder has slaveIndex of -1");
  }
  union bit32 return_byte = get_input_bit32(this->slave_index_, byte_offset);
  return return_byte.i;
}

double Encoder::getAngleRad(uint8_t byte_offset) const
{
  return this->toRad(Encoder::getAngleIU(byte_offset));
}

size_t Encoder::getTotalPositions() const
{
  return this->total_positions_;
}

int Encoder::getSlaveIndex() const
{
  return this->slave_index_;
}

void Encoder::setSlaveIndex(int slave_index)
{
  this->slave_index_ = slave_index;
}

size_t Encoder::calculateTotalPositions(size_t number_of_bits)
{
  if (number_of_bits < Encoder::MIN_RESOLUTION || number_of_bits > Encoder::MAX_RESOLUTION)
  {
    throw error::HardwareException(error::ErrorType::INVALID_ENCODER_RESOLUTION,
                                   "Encoder resolution of %d is not within range [%ld, %ld]", number_of_bits,
                                   Encoder::MIN_RESOLUTION, Encoder::MAX_RESOLUTION);
  }
  return 1u << number_of_bits;
}

}  // namespace march
