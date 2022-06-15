#include "march_hardware/ethercat/sdo_interface.h"

#include <cstdint>

#include <ros/ros.h>
#include <soem/ethercat.h>

namespace march {
int SdoInterfaceImpl::write(
    uint16_t slave, uint16_t index, uint8_t sub, std::size_t size, void* value)
{
    ROS_DEBUG("sdo_write: slave %i, reg 0x%X, sub index %i", slave, index, sub);
    const int working_counter
        = ec_SDOwrite(slave, index, sub, FALSE, size, value, EC_TIMEOUTRXM);
    if (working_counter == 0) {
        ROS_FATAL("sdo_write: Error occurred when writing: slave %i, reg 0x%X, "
                  "sub index %i",
            slave, index, sub);
    }
    return working_counter;
}

int SdoInterfaceImpl::read(uint16_t slave, uint16_t index, uint8_t sub,
    int& val_size, void* value) const
{
    ROS_DEBUG("sdo_read: slave %i, reg 0x%X, sub index %i", slave, index, sub);
    const int working_counter
        = ec_SDOread(slave, index, sub, FALSE, &val_size, value, EC_TIMEOUTRXM);
    if (working_counter == 0) {
        ROS_FATAL("sdo_read: Error occurred when reading: slave %i, reg 0x%X, "
                  "sub index %i",
            slave, index, sub);
    }
    return working_counter;
}
} // namespace march
