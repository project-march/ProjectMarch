// Copyright 2019 Project March.

#ifndef MARCH_HARDWARE_ENCODER_H
#define MARCH_HARDWARE_ENCODER_H
#include "march_hardware/ethercat/pdo_interface.h"

#include <cmath>
#include <cstddef>
#include <cstdint>

namespace march {
class Encoder {
public:
  explicit Encoder(size_t number_of_bits);

  virtual ~Encoder() noexcept = default;

  /**
   * Converts encoder Internal Units (IU) to radians.
   * This is a pure virtual function and must be implemented by subclasses,
   * since each type of encoder has a different way of calculating radians.
   * Conversion may be different if it is position or velocity
   * @param use_zero_position: Whether to use the zero position in the calculation
   */
  virtual double toRadians(double iu, bool use_zero_position) const = 0;

  /**
   * Converts radians to encoder Internal Units (IU).
   * This is a pure virtual function and must be implemented by subclasses,
   * since each type of encoder has a different way of calculating radians.
   * Conversion may be different if it is position or velocity
   * @param use_zero_position: Whether to use the zero position in the calculation
   */
  virtual double toIU(double radians, bool use_zero_position) const = 0;

  /**
   * Returns the amount of radians corresponding to a single LSB flip.
   *
   * Say for example an absolute encoder has a resolution of 12 bits.
   * Then there are 2^12 = 4096 different positions that can be encoded.
   * A complete circle is 2 PI radians.
   * Hence to encode a complete circle each position must account for 2 PI / 4096 radians.
   * In this case flipping the LSB results in a position difference of 0.00153.
   * This value can be used to convert from IU to radians and back.
   *
   * This is a pure virtual function and must be implemented by subclasses,
   * since each type of encoder has a different way of calculating radians.
   */
  virtual double getRadiansPerBit() const = 0;

  size_t getTotalPositions() const;

  static const size_t MIN_RESOLUTION = 1;
  static const size_t MAX_RESOLUTION = 32;

  static constexpr double PI_2 = 2 * M_PI;

private:
    /**
     * Returns the total number of positions possible on an encoder
     * with the given amount of bits.
     * @param number_of_bits The resolution of the encoder
     * @returns The total amount of different positions
     * @throws HardwareException When the given resolution is outside the
     * allowed range Which is determined by Encoder::MIN_RESOLUTION and
     * Encoder::MAX_RESOLUTION.
     */
    static size_t calculateTotalPositions(size_t number_of_bits);

    size_t total_positions_ = 0;
};
} // namespace march

#endif // MARCH_HARDWARE_ENCODER_H
