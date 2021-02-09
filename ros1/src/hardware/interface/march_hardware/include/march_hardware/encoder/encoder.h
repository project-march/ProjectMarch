// Copyright 2019 Project March.

#ifndef MARCH_HARDWARE_ENCODER_H
#define MARCH_HARDWARE_ENCODER_H
#include "march_hardware/communication/ethercat/pdo_interface.h"

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
   * @param pdo PDO interface to use for reading
   * @param byte_offset the byte offset in the slave register for the IU position
   * @returns The current position of the encoder in Internal Units (IU)
   */
  int32_t getAngleIU(const PdoSlaveInterface& pdo, uint8_t byte_offset) const;

  /**
   * Reads out the encoder from the slave and transforms the result to radians.
   * @param pdo PDO interface to use for reading
   * @param byte_offset the byte offset in the slave register for the IU position
   * @returns The current position of the encoder in radians
   */
  double getAngleRad(const PdoSlaveInterface& pdo, uint8_t byte_offset) const;

  /**
   * Reads out the velocity of the encoder from the slave and returns the value in Internal Units per second (IU/s).
   * @param pdo PDO interface to use for reading
   * @param byte_offset the byte offset in the slave register for the IU velocity
   * @returns The current velocity measured by the encoder in Internal Units per second (IU/s)
   */
  double getVelocityIU(const PdoSlaveInterface& pdo, uint8_t byte_offset) const;

  /**
   * Reads out the velocity of the encoder from the slave and transforms the result to radians per second.
   * @param pdo PDO interface to use for reading
   * @param byte_offset the byte offset in the slave register for the IU velocity
   * @returns The current velocity measured by the encoder in radians per second
   */
  double getVelocityRad(const PdoSlaveInterface& pdo, uint8_t byte_offset) const;

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

  static const size_t MIN_RESOLUTION = 1;
  static const size_t MAX_RESOLUTION = 32;

  // constant used for converting a fixed point 16.16 bit number to a double, which is done by dividing by 2^16
  static constexpr double FIXED_POINT_TO_FLOAT_CONVERSION = 1 << 16;

  // iMOTIONCUBE setting (slow loop sample period)
  static constexpr double TIME_PER_VELOCITY_SAMPLE = 0.004;

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

  size_t total_positions_ = 0;
};
}  // namespace march

#endif  // MARCH_HARDWARE_ENCODER_H
