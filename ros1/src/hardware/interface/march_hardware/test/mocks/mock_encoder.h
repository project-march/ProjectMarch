#pragma once
#include "march_hardware/encoder/encoder.h"

#include <gmock/gmock.h>

class MockEncoder : public march::Encoder {
public:
    explicit MockEncoder(size_t number_of_bits)
        : Encoder(number_of_bits)
    {
    }

    MOCK_CONST_METHOD0(getRadiansPerIU, double());

    MOCK_CONST_METHOD1(positionIUToRadians, double(double));
    MOCK_CONST_METHOD1(velocityIUToRadians, double(double));
    MOCK_CONST_METHOD1(positionRadiansToIU, double(double));
    MOCK_CONST_METHOD1(velocityRadiansToIU, double(double));
};
