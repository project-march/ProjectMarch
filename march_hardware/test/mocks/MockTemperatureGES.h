#pragma once
#include "MockSlave.h"

#include "march_hardware/TemperatureGES.h"

#include <gmock/gmock.h>

class MockTemperatureGES : public march::TemperatureGES
{
public:
  MockTemperatureGES() : TemperatureGES(MockSlave(), 0)
  {
  }

  MOCK_CONST_METHOD0(getTemperature, float());

  MOCK_METHOD2(initSdo, bool(march::SdoInterface& sdo, int cycle_time));
};
