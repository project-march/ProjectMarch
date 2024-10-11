// Copyright 2019 Project March.
#include "march_hardware/encoder/absolute_encoder.h"
#include "march_hardware/error/hardware_exception.h"
#include "march_hardware/motor_controller/motor_controller_type.h"

namespace march {
AbsoluteEncoder::AbsoluteEncoder(size_t counts_per_rotation, MotorControllerType motor_controller_type,
    Direction direction, int32_t lower_limit_iu, int32_t upper_limit_iu, int32_t zero_position_iu,
    double lower_soft_limit_rad_diff, double upper_soft_limit_rad_diff)
    : Encoder(counts_per_rotation, motor_controller_type, direction)
    , zero_position_iu_(zero_position_iu)
    , lower_hard_limit_iu_(lower_limit_iu)
    , upper_hard_limit_iu_(upper_limit_iu)
{

    radians_per_iu_ = calculateRadiansPerIU();
    lower_soft_limit_iu_ = lower_limit_iu + lower_soft_limit_rad_diff / getRadiansPerIU();
    upper_soft_limit_iu_ = upper_limit_iu - upper_soft_limit_rad_diff / getRadiansPerIU();

    inputSanityCheck(lower_soft_limit_rad_diff, upper_soft_limit_rad_diff);
}

AbsoluteEncoder::AbsoluteEncoder(size_t counts_per_rotation, MotorControllerType motor_controller_type,
    int32_t lower_limit_iu, int32_t upper_limit_iu, int32_t zero_position_iu, double lower_soft_limit_rad_diff, double upper_soft_limit_rad_diff)
    : AbsoluteEncoder(counts_per_rotation, motor_controller_type, Direction::Positive, lower_limit_iu, upper_limit_iu,
        zero_position_iu, lower_soft_limit_rad_diff, upper_soft_limit_rad_diff)
{
}

void AbsoluteEncoder::inputSanityCheck(double lower_soft_limit_rad_diff, double upper_soft_limit_rad_diff) const
{
    if (lower_hard_limit_iu_ >= upper_hard_limit_iu_) {
        throw error::HardwareException(error::ErrorType::INVALID_RANGE_OF_MOTION,
            "Lower HARD limit iu (= %d) should be smaller than upper HARD limit iu (= %d)", lower_hard_limit_iu_,
            upper_hard_limit_iu_);
    }
    if (zero_position_iu_ < lower_hard_limit_iu_ || zero_position_iu_ > upper_hard_limit_iu_) {
        throw error::HardwareException(error::ErrorType::INVALID_RANGE_OF_MOTION,
            "Zero position (= %d iu) should lie between, its lower and upper hard limit ['%d', '%d'].",
            zero_position_iu_, lower_hard_limit_iu_, upper_hard_limit_iu_);
    }
    if (lower_soft_limit_iu_ >= upper_soft_limit_iu_) {
        throw error::HardwareException(error::ErrorType::INVALID_RANGE_OF_MOTION,
            "Lower SOFT limit (= %d iu, = %g rad diff) >= upper SOFT limit (= %d iu, = %g rad diff).",
            lower_soft_limit_iu_, lower_soft_limit_rad_diff, upper_soft_limit_iu_, upper_soft_limit_rad_diff);
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

int32_t AbsoluteEncoder::positionRadiansToIU(double position) const
{
    return (int)(position / getRadiansPerIU()) + zero_position_iu_;
}

bool AbsoluteEncoder::isWithinHardLimitsIU(int32_t iu) const
{
    return (iu >= lower_hard_limit_iu_ && iu <= upper_hard_limit_iu_);
}

bool AbsoluteEncoder::isWithinHardLimitsRadians(double pos_in_radians) const
{
    return isWithinHardLimitsIU(positionRadiansToIU(pos_in_radians));
}

bool AbsoluteEncoder::isWithinSoftLimitsIU(int32_t iu) const
{
    return (iu >= lower_soft_limit_iu_ && iu <= upper_soft_limit_iu_);
}

bool AbsoluteEncoder::isWithinSoftLimitsRadians(double pos_in_radians) const
{
    return isWithinSoftLimitsIU(positionRadiansToIU(pos_in_radians));
}

bool AbsoluteEncoder::isValidTargetIU(int32_t current_iu, int32_t target_iu) const
{
    if (target_iu < lower_soft_limit_iu_) {
        return target_iu >= current_iu;
    }

    if (target_iu > upper_soft_limit_iu_) {
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

int32_t AbsoluteEncoder::getUpperHardLimitIU() const
{
    return upper_hard_limit_iu_;
}

int32_t AbsoluteEncoder::getLowerHardLimitIU() const
{
    return lower_hard_limit_iu_;
}

} // namespace march
