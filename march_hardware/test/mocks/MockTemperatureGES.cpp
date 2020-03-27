#include "march_hardware/TemperatureGES.h"

#include <gmock/gmock.h>

class MockTemperatureGES : public march::TemperatureGES
{
public:
  MockTemperatureGES() : march::TemperatureGES(1, 0)
  {
  }

  MOCK_METHOD1(writeInitialSDOs, void(int));
  MOCK_METHOD0(getTemperature, float());
  MOCK_METHOD0(getSlaveIndex, int());
};
