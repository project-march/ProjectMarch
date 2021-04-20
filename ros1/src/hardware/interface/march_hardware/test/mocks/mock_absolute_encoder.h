#pragma once
#include "march_hardware/encoder/absolute_encoder.h"

#include <gmock/gmock.h>

class MockAbsoluteEncoder : public march::AbsoluteEncoder {
public:
    MockAbsoluteEncoder()
        : AbsoluteEncoder(/*number_of_bits=*/10, /*lower_limit_iu=*/0,
            /*upper_limit_iu=*/162, /*lower_limit_rad=*/0,
            /*upper_limit_rad=*/1, /*lower_soft_limit_rad=*/
            0.1, /*upper_soft_limit_rad=*/0.9)
    {
    }

    MOCK_METHOD0(getAngleIU, int32_t());
    MOCK_METHOD0(getAngleRad, double());
};
