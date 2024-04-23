#ifndef __clang_analyzer__
// NOLINTBEGIN
#pragma once
#include "march_hardware/encoder/incremental_encoder.h"
#include "march_hardware/motor_controller/motor_controller_type.h"

#include <gmock/gmock.h>

class MockIncrementalEncoder : public march::IncrementalEncoder {
public:
    MockIncrementalEncoder()
        : IncrementalEncoder(
            /*counts_per_rotation=*/1 << 10, march::MotorControllerType::IMotionCube, /*transmission=*/100.0)
    {
    }
};
// NOLINTEND
#endif
