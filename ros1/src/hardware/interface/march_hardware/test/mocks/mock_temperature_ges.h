#pragma once
#include "mock_slave.h"

#include "march_hardware/temperature/temperature_ges.h"

#include <gmock/gmock.h>

class MockTemperatureGES : public march::TemperatureGES {
public:
    MockTemperatureGES()
        : TemperatureGES(MockSlave(), 0)
    {
    }

    MOCK_CONST_METHOD0(getTemperature, float());

    MOCK_METHOD2(initSdo, bool(march::SdoSlaveInterface& sdo, int cycle_time));
};
