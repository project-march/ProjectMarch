#pragma once
#include "march_hardware/encoder/encoder.h"
#include "march_hardware/motor_controller/motor_controller_type.h"

#include <gmock/gmock.h>

class MockEncoder : public march::Encoder {
public:
    explicit MockEncoder(size_t resolution)
        : Encoder(resolution, march::MotorControllerType::IMotionCube)
    {
    }

    MOCK_CONST_METHOD0(getRadiansPerIU, double());

    MOCK_CONST_METHOD1(positionIUToRadians, double(double));
    MOCK_CONST_METHOD1(velocityIUToRadians, double(double));
    MOCK_CONST_METHOD1(positionRadiansToIU, double(double));
    MOCK_CONST_METHOD1(velocityRadiansToIU, double(double));
};
