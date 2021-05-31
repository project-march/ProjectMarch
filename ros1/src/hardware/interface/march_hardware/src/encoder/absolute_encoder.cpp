// Copyright 2019 Project March.
#include "march_hardware/encoder/absolute_encoder.h"
#include "march_hardware/error/hardware_exception.h"
#include "march_hardware/motor_controller/motor_controller_type.h"

#include <ros/ros.h>

namespace march {
AbsoluteEncoder::AbsoluteEncoder(size_t resolution,
    MotorControllerType motor_controller_type, Direction direction, int32_t lower_limit_iu,
    int32_t upper_limit_iu, double lower_limit_rad, double /*upper_limit_rad*/,
    double lower_soft_limit_rad, double upper_soft_limit_rad)
    : Encoder(resolution, motor_controller_type, direction)
    , lower_limit_iu_(lower_limit_iu)
    , upper_limit_iu_(upper_limit_iu)
{
    zero_position_iu_ = lower_limit_iu_ - lower_limit_rad / (getRadiansPerIU());

    lower_soft_limit_iu_ = positionRadiansToIU(lower_soft_limit_rad);
    upper_soft_limit_iu_ = positionRadiansToIU(upper_soft_limit_rad);

//    if (lower_limit_iu_ >= upper_limit_iu_
//        || lower_soft_limit_iu_ >= upper_soft_limit_iu_
//        || lower_soft_limit_iu_ < lower_limit_iu_
//        || upper_soft_limit_iu_ > upper_limit_iu_) {
//        throw error::HardwareException(
//            error::ErrorType::INVALID_RANGE_OF_MOTION,
//            "lower_soft_limit: %d IU, upper_soft_limit: %d IU\n"
//            "lower_hard_limit: %d IU, upper_hard_limit: %d IU",
//            lower_soft_limit_iu_, upper_soft_limit_iu_, lower_limit_iu_,
//            upper_limit_iu_);
//    }
//
//    const double range_of_motion = upper_limit_rad - lower_limit_rad;
//    const double encoder_range_of_motion = positionIUToRadians(upper_limit_iu_)
//        - positionIUToRadians(lower_limit_iu_);
//    const double difference
//        = std::abs(encoder_range_of_motion - range_of_motion)
//        / encoder_range_of_motion;
//    if (difference > AbsoluteEncoder::MAX_RANGE_DIFFERENCE) {
//        ROS_WARN("Difference in range of motion of %.2f%% exceeds %.2f%%\n"
//                 "Absolute encoder range of motion = %f rad\n"
//                 "Limits range of motion = %f rad",
//            difference * 100, AbsoluteEncoder::MAX_RANGE_DIFFERENCE * 100,
//            encoder_range_of_motion, range_of_motion);
//    }
}

double AbsoluteEncoder::getRadiansPerIU() const
{
    switch (getMotorControllerType()) {
        case MotorControllerType::IMotionCube:
        case MotorControllerType::ODrive:
            return PI_2 / getTotalPositions();
        default:
            throw error::HardwareException(
                error::ErrorType::INVALID_MOTOR_CONTROLLER);
    }
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
    return (iu > lower_limit_iu_ && iu < upper_limit_iu_);
}

bool AbsoluteEncoder::isWithinSoftLimitsIU(int32_t iu) const
{
    return (iu > lower_soft_limit_iu_ && iu < upper_soft_limit_iu_);
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

int32_t AbsoluteEncoder::getUpperHardLimitIU() const
{
    return upper_limit_iu_;
}

int32_t AbsoluteEncoder::getLowerHardLimitIU() const
{
    return lower_limit_iu_;
}

} // namespace march
