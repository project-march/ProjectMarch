// Copyright 2019 Project March.
#include "march_hardware/power/low_voltage.h"
#include "march_hardware/ethercat/pdo_interface.h"
#include "march_hardware/ethercat/pdo_types.h"

namespace march {
LowVoltage::LowVoltage(PdoSlaveInterface& pdo,
    NetMonitorOffsets netMonitoringOffsets, NetDriverOffsets netDriverOffsets)
    : pdo_(pdo)
    , netMonitoringOffsets(netMonitoringOffsets)
    , netDriverOffsets(netDriverOffsets)
{
}

float LowVoltage::getNetCurrent(int netNumber)
{
    bit32 current = this->pdo_.read32(
        this->netMonitoringOffsets.getLowVoltageNetCurrent(netNumber));
    return current.f;
}

bool LowVoltage::getNetOperational(int netNumber)
{
    if (netNumber < 1 || netNumber > 2) {
        ROS_ERROR_THROTTLE(2,
            "Can't get operational state from low voltage net %d, there are "
            "only 2 low voltage nets",
            netNumber);
        throw std::invalid_argument("Only low voltage net 1 and 2 exist");
    }
    bit8 operational
        = this->pdo_.read8(this->netMonitoringOffsets.getLowVoltageState());
    // The last bit of the 8 bits represents net 1
    // The second to last bit of the 8 bits represents net 2
    return ((operational.ui >> (netNumber - 1)) & 1);
}

void LowVoltage::setNetOnOff(bool /* on */, int /* netNumber */)
{
    ROS_ERROR_THROTTLE(2, "Can't control low voltage nets from master");
}

uint8_t LowVoltage::getNetsOperational()
{
    bit8 operational
        = this->pdo_.read8(this->netMonitoringOffsets.getLowVoltageState());
    return operational.ui;
}

} // namespace march
