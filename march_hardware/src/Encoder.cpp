// Copyright 2019 Project March.
#include "march_hardware/Encoder.h"
#include "march_hardware/EtherCAT/EthercatIO.h"
#include "march_hardware/error/hardware_exception.h"

#include <cmath>

#include <ros/ros.h>

namespace march
{
const double PI_2 = 2 * M_PI;

Encoder::Encoder(size_t number_of_bits, int32_t lower_limit_iu, int32_t upper_limit_iu, double lower_limit_rad,
                 double upper_limit_rad, double lower_soft_limit_rad, double upper_soft_limit_rad)
  : lower_limit_iu_(lower_limit_iu), upper_limit_iu_(upper_limit_iu)
{
  this->total_positions_ = Encoder::calculateTotalPositions(number_of_bits);

  this->zero_position_iu_ = this->lower_limit_iu_ - lower_limit_rad * this->total_positions_ / PI_2;
  this->lower_soft_limit_iu_ = this->fromRad(lower_soft_limit_rad);
  this->upper_soft_limit_iu_ = this->fromRad(upper_soft_limit_rad);

  if (this->lower_limit_iu_ >= this->upper_limit_iu_ || this->lower_soft_limit_iu_ >= this->upper_soft_limit_iu_ ||
      this->lower_soft_limit_iu_ < this->lower_limit_iu_ || this->upper_soft_limit_iu_ > this->upper_limit_iu_)
  {
    throw error::HardwareException(error::ErrorType::INVALID_RANGE_OF_MOTION,
                                   "lower_soft_limit: %d IU, upper_soft_limit: %d IU\n"
                                   "lower_hard_limit: %d IU, upper_hard_limit: %d IU",
                                   this->lower_soft_limit_iu_, this->upper_soft_limit_iu_, this->lower_limit_iu_,
                                   this->upper_limit_iu_);
  }

  const double range_of_motion = upper_limit_rad - lower_limit_rad;
  const double encoder_range_of_motion = this->toRad(this->upper_limit_iu_) - this->toRad(this->lower_limit_iu_);
  const double difference = std::abs(encoder_range_of_motion - range_of_motion) / encoder_range_of_motion;
  if (difference > Encoder::MAX_RANGE_DIFFERENCE)
  {
    ROS_WARN("Difference in range of motion of %.2f%% exceeds %.2f%%\n"
             "Encoder range of motion = %f rad\n"
             "Limits range of motion = %f rad",
             difference * 100, Encoder::MAX_RANGE_DIFFERENCE * 100, encoder_range_of_motion, range_of_motion);
  }
}

double Encoder::getAngleRad(uint8_t actual_position_byte_offset) const
{
  return this->toRad(this->getAngleIU(actual_position_byte_offset));
}

int32_t Encoder::getAngleIU(uint8_t actual_position_byte_offset) const
{
  if (this->slave_index_ == -1)
  {
    ROS_FATAL("Encoder has slaveIndex of -1");
  }
  union bit32 return_byte = get_input_bit32(this->slave_index_, actual_position_byte_offset);
  ROS_DEBUG("Encoder read (IU): %d", return_byte.i);
  return return_byte.i;
}

int32_t Encoder::fromRad(double rad) const
{
  return (rad * this->total_positions_ / PI_2) + this->zero_position_iu_;
}

double Encoder::toRad(int32_t iu) const
{
  return (iu - this->zero_position_iu_) * PI_2 / this->total_positions_;
}

bool Encoder::isWithinHardLimitsIU(int32_t iu) const
{
  return (iu > this->lower_limit_iu_ && iu < this->upper_limit_iu_);
}

bool Encoder::isWithinSoftLimitsIU(int32_t iu) const
{
  return (iu > this->lower_soft_limit_iu_ && iu < this->upper_soft_limit_iu_);
}

bool Encoder::isValidTargetIU(int32_t current_iu, int32_t target_iu) const
{
  if (target_iu <= this->lower_soft_limit_iu_)
  {
    return target_iu >= current_iu;
  }

  if (target_iu >= this->upper_soft_limit_iu_)
  {
    return target_iu <= current_iu;
  }

  return true;
}

void Encoder::setSlaveIndex(int slave_index)
{
  this->slave_index_ = slave_index;
}

int32_t Encoder::getUpperSoftLimitIU() const
{
  return this->upper_soft_limit_iu_;
}

int32_t Encoder::getLowerSoftLimitIU() const
{
  return this->lower_soft_limit_iu_;
}

int32_t Encoder::getUpperHardLimitIU() const
{
  return this->upper_limit_iu_;
}

int32_t Encoder::getLowerHardLimitIU() const
{
  return this->lower_limit_iu_;
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
