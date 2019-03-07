#include "gmock/gmock.h"  // Brings in Google Mock.
#include "march_hardware/TemperatureSensor.h"

class MockTemperatureSensor : public march4cpp::TemperatureSensor
{
public:
  MOCK_METHOD0(getTemperature, float());
};