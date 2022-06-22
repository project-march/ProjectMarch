#pragma once
#include "march_hardware/encoder/absolute_encoder.h"
#include "march_hardware/motor_controller/motor_controller_type.h"

#include <gmock/gmock.h>

class MockAbsoluteEncoder : public march::AbsoluteEncoder {
public:
    MockAbsoluteEncoder()
        : AbsoluteEncoder(/*resolution=*/10,
            march::MotorControllerType::IMotionCube, /*lower_limit_iu=*/0,
            /*upper_limit_iu=*/162, /*lower_limit_rad=*/0,
            /*upper_limit_rad=*/1, /*lower_soft_limit_rad=*/
            0.1, /*upper_soft_limit_rad=*/0.9)
    {
    }
};
