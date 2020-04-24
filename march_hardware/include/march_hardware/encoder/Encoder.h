// Copyright 2019 Project March.

#ifndef MARCH_HARDWARE_ENCODER_H
#define MARCH_HARDWARE_ENCODER_H
#include <cmath>
#include <cstddef>
#include <cstdint>

namespace march
{
class Encoder
{
public:
  explicit Encoder(size_t number_of_bits);

  virtual ~Encoder() noexcept = default;

  /**
   * Reads out the encoder from the slave and returns the value in Internal Units (IU).
   * @param byte_offset the byte offset in the slave register for the IU position
   * @returns The current position of the encoder in Internal Units (IU)
   */
  int32_t getAngleIU(uint8_t byte_offset) const;

  /**
   * Reads out the encoder from the slave and transforms the result to radians.
   * @param byte_offset the byte offset in the slave register for the IU position
   * @returns The current position of the encoder in radians
   */
  double getAngleRad(uint8_t byte_offset) const;

  /**
   * Converts encoder Internal Units (IU) to radians.
   * This is a pure virtual function and must be implemented by subclasses,
   * since each type of encoder has a different way of calculating radians.
   */
  virtual double toRad(int32_t iu) const = 0;

  /**
   * Returns the radians corresponding to the distance between two bits.
   * This is a pure virtual function and must be implemented by subclasses,
   * since each type of encoder has a different way of calculating radians.
   */
  virtual double getRadPerBit() const = 0;

  size_t getTotalPositions() const;

  int getSlaveIndex() const;
  void setSlaveIndex(int slave_index);

  static const size_t MIN_RESOLUTION = 1;
  static const size_t MAX_RESOLUTION = 32;

  static constexpr double PI_2 = 2 * M_PI;

private:
  /**
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
};
}  // namespace march

#endif  // MARCH_HARDWARE_ENCODER_H
