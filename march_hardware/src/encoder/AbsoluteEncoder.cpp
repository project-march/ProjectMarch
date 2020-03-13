// Copyright 2019 Project March.
#include "march_hardware/encoder/AbsoluteEncoder.h"
#include "march_hardware/error/hardware_exception.h"

#include <ros/ros.h>

namespace march
{
AbsoluteEncoder::AbsoluteEncoder(size_t number_of_bits, int32_t lower_limit_iu, int32_t upper_limit_iu,
                                 double lower_limit_rad, double upper_limit_rad, double lower_soft_limit_rad,
                                 double upper_soft_limit_rad)
  : Encoder(number_of_bits), lower_limit_iu_(lower_limit_iu), upper_limit_iu_(upper_limit_iu)
{
  this->zero_position_iu_ = this->lower_limit_iu_ - lower_limit_rad * this->getTotalPositions() / PI_2;
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
  if (difference > AbsoluteEncoder::MAX_RANGE_DIFFERENCE)
  {
    ROS_WARN("Difference in range of motion of %.2f%% exceeds %.2f%%\n"
             "Absolute encoder range of motion = %f rad\n"
             "Limits range of motion = %f rad",
             difference * 100, AbsoluteEncoder::MAX_RANGE_DIFFERENCE * 100, encoder_range_of_motion, range_of_motion);
  }
}

double AbsoluteEncoder::toRad(int32_t iu) const
{
  return (iu - this->zero_position_iu_) * getRadPerBit();
}

double AbsoluteEncoder::getRadPerBit() const
{
  return PI_2 / this->getTotalPositions();
}

int32_t AbsoluteEncoder::fromRad(double rad) const
{
  return (rad * this->getTotalPositions() / PI_2) + this->zero_position_iu_;
}

bool AbsoluteEncoder::isWithinHardLimitsIU(int32_t iu) const
{
  return (iu > this->lower_limit_iu_ && iu < this->upper_limit_iu_);
}

bool AbsoluteEncoder::isWithinSoftLimitsIU(int32_t iu) const
{
  return (iu > this->lower_soft_limit_iu_ && iu < this->upper_soft_limit_iu_);
}

bool AbsoluteEncoder::isValidTargetIU(int32_t current_iu, int32_t target_iu) const
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

int32_t AbsoluteEncoder::getUpperSoftLimitIU() const
{
  return this->upper_soft_limit_iu_;
}

int32_t AbsoluteEncoder::getLowerSoftLimitIU() const
{
  return this->lower_soft_limit_iu_;
}

int32_t AbsoluteEncoder::getUpperHardLimitIU() const
{
  return this->upper_limit_iu_;
}

int32_t AbsoluteEncoder::getLowerHardLimitIU() const
{
  return this->lower_limit_iu_;
}

}  // namespace march
