#ifndef __clang_analyzer__
// NOLINTBEGIN
#pragma once
#include "march_hardware/encoder/encoder.h"
#include "march_hardware/motor_controller/motor_controller_type.h"

#include <gmock/gmock.h>

class MockEncoder : public march::Encoder {
public:
    explicit MockEncoder(size_t counts_per_rotation)
        : Encoder(counts_per_rotation, march::MotorControllerType::IMotionCube)
    {
    }

    double calculateRadiansPerIU() const final
    {
        return 0.0;
    }

    MOCK_CONST_METHOD1(positionIUToRadians, double(double));
    MOCK_CONST_METHOD1(velocityIUToRadians, double(double));
    MOCK_CONST_METHOD1(positionRadiansToIU, int32_t(double));
    MOCK_CONST_METHOD1(velocityRadiansToIU, double(double));
};
// NOLINTEND
#endif
