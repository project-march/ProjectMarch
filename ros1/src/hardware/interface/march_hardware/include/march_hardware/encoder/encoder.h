// Copyright 2019 Project March.

#ifndef MARCH_HARDWARE_ENCODER_H
#define MARCH_HARDWARE_ENCODER_H
#include "march_hardware/ethercat/pdo_interface.h"
#include "march_hardware/motor_controller/motor_controller_type.h"

#include <cmath>
#include <cstddef>
#include <cstdint>

namespace march {
class Encoder {
public:
    enum Direction { Positive = 1, Negative = -1 };
    Encoder(size_t resolution, MotorControllerType motor_controller_type,
        Direction direction);

    virtual ~Encoder() noexcept = default;

    /**
     * Convert encoder Internal Units (IU) to radians.
     * Conversion may be different if it is position or velocity.
     * These methods may be overridden by child classes.
     */
    virtual double positionIUToRadians(double position) const;
    virtual double velocityIUToRadians(double velocity) const;

    /**
     * Convert radians to encoder Internal Units (IU).
     * Conversion may be different if it is position or velocity.
     * These methods may be overridden by child classes.
     */
    virtual double positionRadiansToIU(double position) const;
    virtual double velocityRadiansToIU(double velocity) const;

    /**
     * Returns the amount of radians corresponding to a single Internal Unit
     *
     * Say for example an absolute encoder has a resolution of 12 bits.
     * Then there are 2^12 = 4096 different positions that can be encoded.
     * A complete circle is 2 PI radians.
     * Hence to encode a complete circle each position must account for 2 PI /
     * 4096 radians. In this case adding a single IU results in a position
     * difference of 0.00153. This value can be used to convert from IU to
     * radians and back.
     *
     * This is a pure virtual function and must be implemented by subclasses,
     * since each type of encoder has a different way of calculating radians.
     */
    virtual double getRadiansPerIU() const = 0;

    size_t getTotalPositions() const;
    Direction getDirection() const;
    MotorControllerType getMotorControllerType() const;

    static const size_t MIN_RESOLUTION = 1;
    static const size_t MAX_RESOLUTION = 32;

    static constexpr double PI_2 = 2 * M_PI;

private:
    /**
     * Returns the total number of positions possible on an encoder
     * with the given amount of bits.
     * @param resolution The resolution of the encoder
     * @returns The total amount of different positions
     * @throws HardwareException When the given resolution is outside the
     * allowed range Which is determined by Encoder::MIN_RESOLUTION and
     * Encoder::MAX_RESOLUTION.
     */
    static size_t calculateTotalPositions(size_t resolution);

    size_t total_positions_ = 0;
    MotorControllerType motor_controller_type_;

    // Not used in calculations, only when reading values
    Direction direction_;
};
} // namespace march

#endif // MARCH_HARDWARE_ENCODER_H
