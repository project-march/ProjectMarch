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
        AbsoluteEncoder(size_t counts_per_rotation, MotorControllerType motor_controller_type, Direction direction,
            int32_t lower_limit_iu, int32_t upper_limit_iu, int32_t zero_position_iu,
            double lower_soft_limit_rad_diff, double upper_soft_limit_rad_diff);

        AbsoluteEncoder(size_t counts_per_rotation, MotorControllerType motor_controller_type, int32_t lower_limit_iu,
            int32_t upper_limit_iu, int32_t zero_position_iu, double lower_soft_limit_rad_diff, double upper_soft_limit_rad_diff);

        ~AbsoluteEncoder() noexcept override = default;

        // Inherited methods
        double calculateRadiansPerIU() const final;
        double positionIUToRadians(double position) const final;
        int32_t positionRadiansToIU(double position) const final;

        bool isWithinHardLimitsIU(int32_t iu) const;
        bool isWithinHardLimitsRadians(double pos_in_radians) const;
        bool isWithinSoftLimitsIU(int32_t iu) const;
        bool isWithinSoftLimitsRadians(double pos_in_radians) const;
        bool isValidTargetIU(int32_t current_iu, int32_t target_iu) const;

        int32_t getUpperSoftLimitIU() const;
        int32_t getLowerSoftLimitIU() const;
        int32_t getUpperHardLimitIU() const;
        int32_t getLowerHardLimitIU() const;

        /** @brief Override comparison operator */
        friend bool operator==(const AbsoluteEncoder& lhs, const AbsoluteEncoder& rhs)
        {
            return lhs.getTotalPositions() == rhs.getTotalPositions()
                && lhs.lower_soft_limit_iu_ == rhs.lower_soft_limit_iu_
                && lhs.upper_soft_limit_iu_ == rhs.upper_soft_limit_iu_
                && lhs.zero_position_iu_ == rhs.zero_position_iu_
                && lhs.lower_hard_limit_iu_ == rhs.lower_hard_limit_iu_ 
                && lhs.upper_hard_limit_iu_ == rhs.upper_hard_limit_iu_;
        }

        /** @brief Override stream operator for clean printing */
        friend std::ostream& operator<<(std::ostream& os, const AbsoluteEncoder& encoder)
        {
            return os << "totalPositions: " << encoder.getTotalPositions() << ", "
                    << "lowerHardLimit: " << encoder.lower_hard_limit_iu_ << ", "
                    << "upperHardLimit: " << encoder.upper_hard_limit_iu_ << ", "
                    << "zeroPositionIU: " << encoder.zero_position_iu_ << ", "
                    << "upperSoftLimit: " << encoder.upper_soft_limit_iu_ << ", "
                    << "lowerSoftLimit: " << encoder.lower_soft_limit_iu_; 
        }

    private:
        int32_t zero_position_iu_ = 0;
        int32_t lower_soft_limit_iu_ = 0;
        int32_t upper_soft_limit_iu_ = 0;
        int32_t lower_hard_limit_iu_ = 0;
        int32_t upper_hard_limit_iu_ = 0;

        // Checks if the limits are defined correctly.
        void inputSanityCheck(double lower_soft_limit_rad_diff, double upper_soft_limit_rad_diff) const;
    };

} // namespace march

#endif // MARCH_HARDWARE_ABSOLUTE_ENCODER_H
