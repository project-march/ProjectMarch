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

bool LowVoltage::isValidLowVoltageNetNumber(uint8_t netNumber)
{
    return netNumber >= MIN_NET_NUMBER && netNumber <= MAX_NET_NUMBER;
}

void LowVoltage::throwInvalidNetArgument(
    uint8_t netNumber, const char* caller_name)
{
    ROS_FATAL_THROTTLE(2,
        "Can't execute call at %s for net %d because there are only %d low "
        "voltage nets.",
        caller_name, netNumber, MAX_NET_NUMBER);

    throw std::invalid_argument("Low voltage net number should be between "
        + std::to_string(MIN_NET_NUMBER) + " and "
        + std::to_string(MAX_NET_NUMBER) + ".");
}
float LowVoltage::getNetCurrent(uint8_t netNumber)
{
    if (not isValidLowVoltageNetNumber(netNumber)) {
        throwInvalidNetArgument(netNumber);
    }
    bit32 current = this->pdo_.read32(
        this->netMonitoringOffsets.getLowVoltageNetCurrent(netNumber));
    return current.f;
}

bool LowVoltage::getNetOperational(uint8_t netNumber)
{
    if (not isValidLowVoltageNetNumber(netNumber)) {
        throwInvalidNetArgument(netNumber);
    }
    bit8 operational
        = this->pdo_.read8(this->netMonitoringOffsets.getLowVoltageState());
    // The last bit of the 8 bits represents net 1
    // The second to last bit of the 8 bits represents net 2
    return ((uint8_t)(operational.ui >> (netNumber - 1U)) & 1U);
}

void LowVoltage::setNetOnOff(bool /* on */, uint8_t /* netNumber */)
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
