#include "march_hardware/encoder/Encoder.h"

#include <gmock/gmock.h>

class MockEncoder : public march::Encoder
{
public:
  explicit MockEncoder(size_t number_of_bits) : Encoder(number_of_bits)
  {
  }

  MOCK_CONST_METHOD1(toRad, double(int32_t));

  MOCK_CONST_METHOD0(getRadPerBit, double());
};
