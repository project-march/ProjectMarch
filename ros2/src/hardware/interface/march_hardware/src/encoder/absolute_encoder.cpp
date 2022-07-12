// Copyright 2019 Project March.
#include "march_hardware/encoder/absolute_encoder.h"
#include "march_hardware/error/hardware_exception.h"
#include "march_hardware/motor_controller/motor_controller_type.h"

namespace march {
    AbsoluteEncoder::AbsoluteEncoder(size_t resolution,
                                     MotorControllerType motor_controller_type, Direction direction,
                                     int32_t lower_limit_iu, int32_t upper_limit_iu, int32_t zero_position_iu,
                                     double lower_error_soft_limit_rad_diff, double upper_error_soft_limit_rad_diff,
                                     double lower_soft_limit_rad_diff, double upper_soft_limit_rad_diff)
            : Encoder(resolution, motor_controller_type, direction),
              zero_position_iu_(zero_position_iu),
              lower_hard_limit_iu_(lower_limit_iu), upper_hard_limit_iu_(upper_limit_iu) {

        radians_per_iu_ = calculateRadiansPerIU();
        lower_error_soft_limit_iu_ = lower_limit_iu + lower_error_soft_limit_rad_diff / getRadiansPerIU();
        upper_error_soft_limit_iu_ = upper_limit_iu - upper_error_soft_limit_rad_diff / getRadiansPerIU();
        lower_soft_limit_iu_ = lower_limit_iu + lower_soft_limit_rad_diff / getRadiansPerIU();
        upper_soft_limit_iu_ = upper_limit_iu - upper_soft_limit_rad_diff / getRadiansPerIU();

        inputSanityCheck(lower_error_soft_limit_rad_diff, upper_error_soft_limit_rad_diff,
                         lower_soft_limit_rad_diff, upper_soft_limit_rad_diff);

}

    AbsoluteEncoder::AbsoluteEncoder(size_t resolution,
                                     MotorControllerType motor_controller_type,
                                     int32_t lower_limit_iu, int32_t upper_limit_iu, int32_t zero_position_iu,
                                     double lower_error_soft_limit_rad_diff, double upper_error_soft_limit_rad_diff,
                                     double lower_soft_limit_rad_diff, double upper_soft_limit_rad_diff)
            : AbsoluteEncoder(resolution, motor_controller_type, Direction::Positive,
                              lower_limit_iu, upper_limit_iu, zero_position_iu,
                              lower_error_soft_limit_rad_diff,
                              upper_error_soft_limit_rad_diff, lower_soft_limit_rad_diff, upper_soft_limit_rad_diff) {
    }


    void AbsoluteEncoder::inputSanityCheck(double lower_error_soft_limit_rad_diff, double upper_error_soft_limit_rad_diff,
                                           double lower_soft_limit_rad_diff, double upper_soft_limit_rad_diff) const {
        if (lower_hard_limit_iu_ >= upper_hard_limit_iu_) {
            throw error::HardwareException(
                    error::ErrorType::INVALID_RANGE_OF_MOTION,
                    "Lower HARD limit iu (= %d) should be smaller than upper HARD limit iu (= %d)",
                    lower_hard_limit_iu_, upper_hard_limit_iu_);
        }
        if (zero_position_iu_ < lower_hard_limit_iu_ || zero_position_iu_ > upper_hard_limit_iu_) {
            throw error::HardwareException(
                    error::ErrorType::INVALID_RANGE_OF_MOTION,
                    "Zero position (= %d iu) should lie between, its lower and upper hard limit ['%d', '%d'].",
                    zero_position_iu_, lower_hard_limit_iu_, upper_hard_limit_iu_);
        }
        if (lower_soft_limit_rad_diff > lower_error_soft_limit_rad_diff) {
            throw error::HardwareException(
                    error::ErrorType::INVALID_RANGE_OF_MOTION,
                    "Lower SOFT limit difference (= %d rad) is bigger than "
                    "lower ERROR_SOFT limit difference (= %d rad). "
                    "Error soft limits should lie between (inclusive) the soft and hard limits. ",
                    lower_soft_limit_rad_diff, lower_error_soft_limit_rad_diff);
        }
        if (upper_soft_limit_rad_diff > upper_error_soft_limit_rad_diff) {
            throw error::HardwareException(
                    error::ErrorType::INVALID_RANGE_OF_MOTION,
                    "Upper SOFT limit difference (= %d rad) is bigger than "
                    "upper ERROR_SOFT limit difference (= %d rad). "
                    "Error soft limits should lie between (inclusive) the soft and hard limits. ",
                    upper_soft_limit_rad_diff, upper_error_soft_limit_rad_diff);
        }
        if (lower_soft_limit_iu_ >= upper_soft_limit_iu_) {
            throw error::HardwareException(
                    error::ErrorType::INVALID_RANGE_OF_MOTION,
                    "Lower SOFT limit (= %d iu, = %d rad diff) >= upper SOFT limit (= %d iu, = %d rad diff).",
                    lower_soft_limit_iu_, lower_soft_limit_rad_diff,
                    upper_soft_limit_iu_, upper_soft_limit_rad_diff);
        }
        if (lower_error_soft_limit_iu_ >= upper_error_soft_limit_iu_) {
            throw error::HardwareException(
                    error::ErrorType::INVALID_RANGE_OF_MOTION,
                    "Lower ERROR_SOFT limit (= %d iu, = %d rad diff) "
                    ">= upper ERROR_SOFT limit (= %d iu, = %d rad diff).",
                    lower_error_soft_limit_iu_, lower_error_soft_limit_rad_diff,
                    upper_error_soft_limit_iu_, upper_error_soft_limit_rad_diff);
        }

    }

double AbsoluteEncoder::calculateRadiansPerIU() const
{
    return PI_2 / getTotalPositions();
}

double AbsoluteEncoder::positionIUToRadians(double position) const
{
    return (position - zero_position_iu_) * getRadiansPerIU();
}

double AbsoluteEncoder::positionRadiansToIU(double position) const
{
    return (position / getRadiansPerIU()) + zero_position_iu_;
}

bool AbsoluteEncoder::isWithinHardLimitsIU(int32_t iu) const
{
    return (iu >= lower_hard_limit_iu_ && iu <= upper_hard_limit_iu_);
}

bool AbsoluteEncoder::isWithinHardLimitsRadians(double pos_in_radians) const {
    return isWithinHardLimitsIU(positionRadiansToIU(pos_in_radians));
}

bool AbsoluteEncoder::isWithinSoftLimitsIU(int32_t iu) const
{
    return (iu >= lower_soft_limit_iu_ && iu <= upper_soft_limit_iu_);
}

bool AbsoluteEncoder::isWithinSoftLimitsRadians(double pos_in_radians) const {
    return isWithinSoftLimitsIU(positionRadiansToIU(pos_in_radians));
}

bool AbsoluteEncoder::isWithinErrorSoftLimitsIU(int32_t iu) const
{
    return (iu >= lower_error_soft_limit_iu_ && iu <= upper_error_soft_limit_iu_);
}

bool AbsoluteEncoder::isWithinErrorSoftLimitsRadians(double pos_in_radians) const {
    return isWithinErrorSoftLimitsIU(positionRadiansToIU(pos_in_radians));
}


bool AbsoluteEncoder::isValidTargetIU(
    int32_t current_iu, int32_t target_iu) const
{
    if (target_iu <= lower_soft_limit_iu_) {
        return target_iu >= current_iu;
    }

    if (target_iu >= upper_soft_limit_iu_) {
        return target_iu <= current_iu;
    }

    return true;
}

int32_t AbsoluteEncoder::getUpperSoftLimitIU() const
{
    return upper_soft_limit_iu_;
}

int32_t AbsoluteEncoder::getLowerSoftLimitIU() const
{
    return lower_soft_limit_iu_;
}

int32_t AbsoluteEncoder::getUpperErrorSoftLimitIU() const
{
    return upper_soft_limit_iu_;
}

int32_t AbsoluteEncoder::getLowerErrorSoftLimitIU() const
{
    return lower_soft_limit_iu_;
}

int32_t AbsoluteEncoder::getUpperHardLimitIU() const
{
    return upper_hard_limit_iu_;
}

int32_t AbsoluteEncoder::getLowerHardLimitIU() const
{
    return lower_hard_limit_iu_;
}

} // namespace march
