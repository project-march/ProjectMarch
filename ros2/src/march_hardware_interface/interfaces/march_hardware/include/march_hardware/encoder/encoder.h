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
    Encoder(size_t counts_per_rotation, MotorControllerType motor_controller_type, Direction direction);
    Encoder(size_t counts_per_rotation, MotorControllerType motor_controller_type);

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
    virtual int32_t positionRadiansToIU(double position) const;
    virtual double velocityRadiansToIU(double velocity) const;

    /**
     * Returns the amount of radians corresponding to a single Internal Unit
     *
     * Say for example an absolute encoder has a 4096 counts per rotation.
     * Then there are 4096 different positions that can be encoded.
     * A complete circle is 2 PI radians.
     * Hence to encode a complete circle each position must account for 2 PI /
     * 4096 radians. In this case adding a single IU results in a position
     * difference of 0.00153. This value can be used to convert from IU to
     * radians and back.
     *
     * This is a pure virtual function and must be implemented by subclasses,
     * since each type of encoder has a different way of calculating radians.
     */
    virtual double calculateRadiansPerIU() const = 0;

    size_t getTotalPositions() const;
    double getRadiansPerIU() const;
    Direction getDirection() const;
    MotorControllerType getMotorControllerType() const;

    static const size_t MIN_COUNTS_PER_ROTATION = (size_t)1 << 1;
    static const size_t MAX_COUNTS_PER_ROTATION = (size_t)1 << 32;

    static constexpr double PI_2 = 2 * M_PI;

protected:
    double radians_per_iu_ = 0.0;

private:
    /**
     * Returns the total number of positions possible on an encoder
     * after checking that the input CPR (counts per rotation) is within the
     * correct range.
     * @param counts_per_rotation The total amount of different positions.
     * @returns The total amount of different positions
     * @throws HardwareException When the given CPR is outside the
     * allowed range Which is determined by Encoder::MIN_COUNTS_PER_ROTATION and
     * Encoder::MAX_COUNTS_PER_ROTATION.
     */
    static size_t calculateTotalPositions(size_t counts_per_rotation);

    size_t total_positions_ = 0;

    // TODO: Remove MotorControllerType from Encoder
    // https://gitlab.com/project-march/march/-/issues/982
    MotorControllerType motor_controller_type_;

    // Not used in calculations, only when reading values
    Direction direction_;
};
} // namespace march

#endif // MARCH_HARDWARE_ENCODER_H
