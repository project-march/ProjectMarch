// Copyright 2019 Project March.

#ifndef MARCH_HARDWARE_ABSOLUTE_ENCODER_H
#define MARCH_HARDWARE_ABSOLUTE_ENCODER_H
#include "march_hardware/encoder/encoder.h"
#include "march_hardware/motor_controller/motor_controller_type.h"

#include <march_logger_cpp/base_logger.hpp>
#include <ostream>

namespace march {
class AbsoluteEncoder : public Encoder {
public:
    /**
     * Construct AbsoluteEncoder with both hard and soft limits.
     */
    AbsoluteEncoder(size_t counts_per_rotation, MotorControllerType motor_controller_type, Direction direction,
        int32_t lower_limit_iu, int32_t upper_limit_iu, int32_t zero_position_iu,
        double lower_error_soft_limit_rad_diff, double upper_error_soft_limit_rad_diff,
        double lower_soft_limit_rad_diff, double upper_soft_limit_rad_diff);
    AbsoluteEncoder(size_t counts_per_rotation, MotorControllerType motor_controller_type, int32_t lower_limit_iu,
        int32_t upper_limit_iu, int32_t zero_position_iu, double lower_error_soft_limit_rad_diff,
        double upper_error_soft_limit_rad_diff, double lower_soft_limit_rad_diff, double upper_soft_limit_rad_diff);

    ~AbsoluteEncoder() noexcept override = default;

    // Inherited methods
    double calculateRadiansPerIU() const final;
    double positionIUToRadians(double position) const final;
    int32_t positionRadiansToIU(double position) const final;

    bool isWithinHardLimitsIU(int32_t iu) const;
    bool isWithinHardLimitsRadians(double pos_in_radians) const;
    bool isWithinSoftLimitsIU(int32_t iu) const;
    bool isWithinSoftLimitsRadians(double pos_in_radians) const;
    bool isWithinErrorSoftLimitsIU(int32_t iu) const;
    bool isWithinErrorSoftLimitsRadians(double pos_in_radians) const;
    bool isValidTargetIU(int32_t current_iu, int32_t target_iu) const;

    int32_t getUpperSoftLimitIU() const;
    int32_t getLowerSoftLimitIU() const;
    int32_t getUpperErrorSoftLimitIU() const;
    int32_t getLowerErrorSoftLimitIU() const;
    int32_t getUpperHardLimitIU() const;
    int32_t getLowerHardLimitIU() const;

    /** @brief Override comparison operator */
    friend bool operator==(const AbsoluteEncoder& lhs, const AbsoluteEncoder& rhs)
    {
        return lhs.getTotalPositions() == rhs.getTotalPositions()
            && lhs.upper_soft_limit_iu_ == rhs.upper_soft_limit_iu_
            && lhs.lower_soft_limit_iu_ == rhs.lower_soft_limit_iu_
            && lhs.lower_error_soft_limit_iu_ == rhs.lower_error_soft_limit_iu_
            && lhs.upper_error_soft_limit_iu_ == rhs.upper_error_soft_limit_iu_
            && lhs.upper_hard_limit_iu_ == rhs.upper_hard_limit_iu_
            && lhs.lower_hard_limit_iu_ == rhs.lower_hard_limit_iu_ && lhs.zero_position_iu_ == rhs.zero_position_iu_;
    }
    /** @brief Override stream operator for clean printing */
    friend std::ostream& operator<<(std::ostream& os, const AbsoluteEncoder& encoder)
    {
        return os << "totalPositions: " << encoder.getTotalPositions() << ", "
                  << "upperHardLimit: " << encoder.upper_hard_limit_iu_ << ", "
                  << "lowerHardLimit: " << encoder.lower_hard_limit_iu_ << ", "
                  << "lowerHardLimit: " << encoder.lower_error_soft_limit_iu_ << ", "
                  << "lowerHardLimit: " << encoder.upper_error_soft_limit_iu_ << ", "
                  << "upperSoftLimit: " << encoder.upper_soft_limit_iu_ << ", "
                  << "lowerSoftLimit: " << encoder.lower_soft_limit_iu_ << ", "
                  << "zeroPositionIU: " << encoder.zero_position_iu_;
    }

    static constexpr double MAX_RANGE_DIFFERENCE = 0.05;

private:
    // The zero position of the joint, in internal units.
    // If reading the absolute encoder of the joint gives this value, then
    // the joint is at exactly 0 degrees.
    // Start at zero and is then set by the constructor of the AbsoluteEncoder
    int32_t zero_position_iu_ = 0;

    // Hard limits
    int32_t lower_hard_limit_iu_ = 0;
    int32_t upper_hard_limit_iu_ = 0;

    // Soft limits
    int32_t lower_error_soft_limit_iu_ = 0;
    int32_t upper_error_soft_limit_iu_ = 0;

    // Soft limits
    int32_t lower_soft_limit_iu_ = 0;
    int32_t upper_soft_limit_iu_ = 0;

    /**
     * \brief Checks if the limits are defined correctly.
     * \attention Should be called after the following values are initialized:
     *              lower_error_soft_limit_iu_,
     *              upper_error_soft_limit_iu_,
     *              lower_soft_limit_iu_,
     *              upper_soft_limit_iu_.
     *
     * @throws A HardwareException if the range of motion is not valid.
     */
    void inputSanityCheck(double lower_error_soft_limit_rad_diff, double upper_error_soft_limit_rad_diff,
        double lower_soft_limit_rad_diff, double upper_soft_limit_rad_diff) const;
};
} // namespace march

#endif // MARCH_HARDWARE_ABSOLUTE_ENCODER_H
