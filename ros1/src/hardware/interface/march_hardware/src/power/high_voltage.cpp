// Copyright 2019 Project March.
#include "march_hardware/power/high_voltage.h"
#include "march_hardware/ethercat/pdo_interface.h"
#include "march_hardware/ethercat/pdo_types.h"

namespace march {
HighVoltage::HighVoltage(PdoSlaveInterface& pdo,
    NetMonitorOffsets netMonitoringOffsets, NetDriverOffsets netDriverOffsets)
    : pdo_(pdo)
    , netMonitoringOffsets(netMonitoringOffsets)
    , netDriverOffsets(netDriverOffsets)
{
}

float HighVoltage::getNetCurrent()
{
    bit32 current = this->pdo_.read32(
        this->netMonitoringOffsets.getHighVoltageNetCurrent());
    return current.f;
}

bool HighVoltage::getNetOperational(int netNumber)
{
    if (netNumber < 1 || netNumber > 8) {
        ROS_ERROR_THROTTLE(2,
            "Can't get operational state from high voltage net %d, there are "
            "only 8 high voltage nets",
            netNumber);
        throw std::invalid_argument("Only high voltage net 1 and 8 exist");
    }
    bit8 operational
        = this->pdo_.read8(this->netMonitoringOffsets.getHighVoltageState());
    // The first bit of the 8 bits represents net 1 and so on till the last 8th
    // bit which represents net 8.
    return ((operational.ui >> (netNumber - 1)) & 1);
}

bool HighVoltage::getOvercurrentTrigger(int netNumber)
{
    if (netNumber < 1 || netNumber > 8) {
        ROS_FATAL_THROTTLE(2,
            "Can't get overcurrent trigger from high voltage net %d, there are "
            "only 8 high voltage nets",
            netNumber);
        throw std::exception();
    }
    bit8 overcurrent = this->pdo_.read8(
        this->netMonitoringOffsets.getHighVoltageOvercurrentTrigger());
    return ((overcurrent.ui >> (netNumber - 1)) & 1);
}

bool HighVoltage::getHighVoltageEnabled()
{
    bit8 highVoltageEnabled
        = this->pdo_.read8(this->netMonitoringOffsets.getHighVoltageEnabled());
    return highVoltageEnabled.ui;
}

void HighVoltage::setNetOnOff(bool on, int netNumber)
{
    if (netNumber < 1 || netNumber > 8) {
        ROS_ERROR_THROTTLE(2,
            "Can't turn high voltage net %d on, only high voltage net 1 to 8 "
            "exist",
            netNumber);
        throw std::invalid_argument("Only high voltage net 1 to 8 exist");
    }
    if (on && getNetOperational(netNumber)) {
        ROS_WARN_THROTTLE(2, "High voltage net %d is already on", netNumber);
    }
    uint8_t currentStateHighVoltageNets = getNetsOperational();
    bit8 highVoltageNets;
    highVoltageNets.ui = 1 << (netNumber - 1);
    if (on) {
        // Force bit of the respective net to one.
        highVoltageNets.ui |= currentStateHighVoltageNets;
    } else {
        // Force bit of the respective net to zero.
        highVoltageNets.ui = ~highVoltageNets.ui;
        highVoltageNets.ui &= currentStateHighVoltageNets;
    }
    this->pdo_.write8(
        this->netDriverOffsets.getHighVoltageNetOnOff(), highVoltageNets);
}

void HighVoltage::enableDisableHighVoltage(bool enable)
{
    if (enable && getHighVoltageEnabled()) {
        ROS_ERROR_THROTTLE(2, "High voltage already enabled");
        throw std::runtime_error("High voltage already enabled");
    } else if (!enable && !getHighVoltageEnabled()) {
        ROS_ERROR_THROTTLE(2, "High voltage already disabled");
        throw std::runtime_error("High voltage already disabled");
    }
    if (enable) {
        ROS_DEBUG_THROTTLE(2, "Trying to enable high voltage from software");
    } else {
        ROS_DEBUG_THROTTLE(2, "Trying to disable high voltage from software");
    }

    bit8 isEnabled;
    isEnabled.ui = enable;
    this->pdo_.write8(
        this->netDriverOffsets.getHighVoltageEnableDisable(), isEnabled);
}

uint8_t HighVoltage::getNetsOperational()
{
    bit8 operational
        = this->pdo_.read8(this->netMonitoringOffsets.getHighVoltageState());
    return operational.ui;
}

} // namespace march
