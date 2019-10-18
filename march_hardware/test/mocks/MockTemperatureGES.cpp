#include "gmock/gmock.h"  // Brings in Google Mock.
#include "march_hardware/TemperatureGES.h"

class MockTemperatureGES : public march4cpp::TemperatureGES
{
public:
  MOCK_METHOD0(getTemperature, float());
  MOCK_METHOD0(getSlaveIndex, int());
};