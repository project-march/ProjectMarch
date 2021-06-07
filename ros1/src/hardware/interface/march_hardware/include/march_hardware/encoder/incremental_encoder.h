// Copyright 2019 Project March.

#ifndef MARCH_HARDWARE_INCREMENTAL_ENCODER_H
#define MARCH_HARDWARE_INCREMENTAL_ENCODER_H
#include "march_hardware/encoder/encoder.h"
#include "march_hardware/motor_controller/motor_controller_type.h"

#include <ostream>

namespace march {
class IncrementalEncoder : public Encoder {
public:
    IncrementalEncoder(size_t resolution,
        MotorControllerType motor_controller_type, Direction direction,
        double transmission);

    ~IncrementalEncoder() noexcept override = default;

    // Inherited methods
    double getRadiansPerIU() const final;
    double velocityRadiansToIU(double velocity) const;
    double velocityIUToRadians(double velocity) const;

    double getTransmission() const;

    /** @brief Override comparison operator */
    friend bool operator==(
        const IncrementalEncoder& lhs, const IncrementalEncoder& rhs)
    {
        return lhs.getTotalPositions() == rhs.getTotalPositions()
            && lhs.transmission_ == rhs.transmission_;
    }
    /** @brief Override stream operator for clean printing */
    friend std::ostream& operator<<(
        std::ostream& os, const IncrementalEncoder& encoder)
    {
        return os << "totalPositions: " << encoder.getTotalPositions() << ", "
                  << "transmission: " << encoder.transmission_;
    }

private:
    const double transmission_;
};
} // namespace march

#endif // MARCH_HARDWARE_INCREMENTAL_ENCODER_H
