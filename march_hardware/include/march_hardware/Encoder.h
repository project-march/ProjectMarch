// Copyright 2019 Project March.

#ifndef MARCH_HARDWARE_ENCODER_H
#define MARCH_HARDWARE_ENCODER_H

#include <ostream>

namespace march
{
class Encoder
{
public:
  Encoder(size_t number_of_bits, int32_t lower_limit_iu, int32_t upper_limit_iu, double lower_limit_rad,
          double upper_limit_rad, double lower_soft_limit_rad, double upper_soft_limit_rad);

  /*
   * Reads out the encoder from the slave and transforms the result to radians.
   * @param actual_position_byte_offset the byte offset in the slave register for the IU position
   * @returns The current position of the encoder in radians
   */
  double getAngleRad(uint8_t actual_position_byte_offset) const;

  /*
   * Reads out the encoder from the slave and returns the value in Internal Units (IU).
   * @param actual_position_byte_offset the byte offset in the slave register for the IU position
   * @returns The current position of the encoder in Internal Units (IU)
   */
  int32_t getAngleIU(uint8_t actual_position_byte_offset) const;

  /*
   * Converts encoder Internal Units (IU) to radians.
   */
  double toRad(int32_t iu) const;

  /*
   * Converts radians to encoder Internal Units (IU).
   */
  int32_t fromRad(double rad) const;

  bool isWithinHardLimitsIU(int32_t iu) const;
  bool isWithinSoftLimitsIU(int32_t iu) const;
  bool isValidTargetIU(int32_t current_iu, int32_t target_iu) const;

  void setSlaveIndex(int slave_index);

  int32_t getUpperSoftLimitIU() const;
  int32_t getLowerSoftLimitIU() const;
  int32_t getUpperHardLimitIU() const;
  int32_t getLowerHardLimitIU() const;

  /** @brief Override comparison operator */
  friend bool operator==(const Encoder& lhs, const Encoder& rhs)
  {
    return lhs.slave_index_ == rhs.slave_index_ && lhs.total_positions_ == rhs.total_positions_ &&
           lhs.upper_soft_limit_iu_ == rhs.upper_soft_limit_iu_ &&
           lhs.lower_soft_limit_iu_ == rhs.lower_soft_limit_iu_ && lhs.upper_limit_iu_ == rhs.upper_limit_iu_ &&
           lhs.lower_limit_iu_ == rhs.lower_limit_iu_ && lhs.zero_position_iu_ == rhs.zero_position_iu_;
  }
  /** @brief Override stream operator for clean printing */
  friend std::ostream& operator<<(std::ostream& os, const Encoder& encoder)
  {
    return os << "slaveIndex: " << encoder.slave_index_ << ", "
              << "totalPositions: " << encoder.total_positions_ << ", "
              << "upperHardLimit: " << encoder.upper_limit_iu_ << ", "
              << "lowerHardLimit: " << encoder.lower_limit_iu_ << ", "
              << "upperSoftLimit: " << encoder.upper_soft_limit_iu_ << ", "
              << "lowerSoftLimit: " << encoder.lower_soft_limit_iu_ << ", "
              << "zeroPositionIU: " << encoder.zero_position_iu_;
  }

  static const size_t MIN_RESOLUTION = 1;
  static const size_t MAX_RESOLUTION = 32;

  static constexpr double MAX_RANGE_DIFFERENCE = 0.05;

private:
  /*
   * Returns the total number of positions possible on an encoder
   * with the given amount of bits.
   * @param number_of_bits The resolution of the encoder
   * @returns The total amount of different positions
   * @throws HardwareException When the given resolution is outside the allowed range
   *                           Which is determined by Encoder::MIN_RESOLUTION
   *                           and Encoder::MAX_RESOLUTION.
   */
  static size_t calculateTotalPositions(size_t number_of_bits);

  int slave_index_ = -1;
  size_t total_positions_ = 0;

  int32_t zero_position_iu_ = 0;
  int32_t lower_limit_iu_ = 0;
  int32_t upper_limit_iu_ = 0;
  int32_t lower_soft_limit_iu_ = 0;
  int32_t upper_soft_limit_iu_ = 0;
};
}  // namespace march

#endif  // MARCH_HARDWARE_ENCODER_H
