// Copyright 2019 Project March.
#ifndef MARCH_HARDWARE_HIGH_VOLTAGE_H
#define MARCH_HARDWARE_HIGH_VOLTAGE_H
#include "march_hardware/ethercat/pdo_interface.h"
#include "net_driver_offsets.h"
#include "net_monitor_offsets.h"

#include <cstdint>
#include <iostream>

namespace march {
class HighVoltage {
private:
    PdoSlaveInterface& pdo_;
    NetMonitorOffsets netMonitoringOffsets;
    NetDriverOffsets netDriverOffsets;

    uint8_t getNetsOperational();

    bool isValidHighVoltageNetNumber(uint8_t netNumber);
    void throwInvalidNetArgument(
        uint8_t netNumber, const char* caller_name = __builtin_FUNCTION());
    const uint8_t MIN_NET_NUMBER = 1;
    const uint8_t MAX_NET_NUMBER = 8;

public:
    HighVoltage(PdoSlaveInterface& pdo, NetMonitorOffsets netMonitoringOffsets,
        NetDriverOffsets netDriverOffsets);

    float getNetCurrent();
    bool getNetOperational(uint8_t netNumber);
    bool getOvercurrentTrigger(uint8_t netNumber);
    bool getHighVoltageEnabled();
    void setNetOnOff(bool on, uint8_t netNumber);
    void enableDisableHighVoltage(bool enable);

    /** @brief Override comparison operator */
    friend bool operator==(const HighVoltage& lhs, const HighVoltage& rhs)
    {
        return lhs.netDriverOffsets == rhs.netDriverOffsets
            && lhs.netMonitoringOffsets == rhs.netMonitoringOffsets;
    }

    /** @brief Override stream operator for clean printing */
    friend ::std::ostream& operator<<(
        std::ostream& os, const HighVoltage& highVoltage)
    {
        return os << "HighVoltage(netMonitoringOffsets: "
                  << highVoltage.netMonitoringOffsets << ", "
                  << "netDriverOffsets: " << highVoltage.netDriverOffsets
                  << ")";
    }
};

} // namespace march
#endif // MARCH_HARDWARE_HIGH_VOLTAGE_H
