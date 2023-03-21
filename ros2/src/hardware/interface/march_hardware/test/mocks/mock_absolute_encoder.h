#ifndef __clang_analyzer__
// NOLINTBEGIN
#pragma once
#include "march_hardware/encoder/absolute_encoder.h"
#include "march_hardware/motor_controller/motor_controller_type.h"

#include <gmock/gmock.h>

class MockAbsoluteEncoder : public march::AbsoluteEncoder {
public:
    MockAbsoluteEncoder()
        : AbsoluteEncoder(/*counts_per_rotation=*/1 << 10, march::MotorControllerType::ODrive,
            /*lower_limit_iu=*/531696,
            /*upper_limit_iu=*/882560, /*zero_position*/ 548304, /*lower_limit_rad=*/0.0174533,
            /*upper_limit_rad=*/0.0174533, /*lower_soft_limit_rad=*/
            0.0349066, /*upper_soft_limit_rad=*/0.0349066)
    {
    }
};
// NOLINTEND
#endif
