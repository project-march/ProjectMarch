#include "march_hardware/ethercat/sdo_interface.h"

#include <cstdint>

#include <soem/ethercat.h>

namespace march {
int SdoInterfaceImpl::write(uint16_t slave, uint16_t index, uint8_t sub, std::size_t size, void* value)
{
    logger_->debug(logger_->fstring("sdo_write: slave %i, reg 0x%X, sub index %i", slave, index, sub));
    const int working_counter = ec_SDOwrite(slave, index, sub, FALSE, size, value, EC_TIMEOUTRXM);
    if (working_counter == 0) {
        logger_->fatal(logger_->fstring(
            "sdo_write: Error occurred when writing: slave %i, reg 0x%X, sub index %i", slave, index, sub));
    }
    return working_counter;
}

int SdoInterfaceImpl::read(uint16_t slave, uint16_t index, uint8_t sub, int& val_size, void* value) const
{
    logger_->debug(logger_->fstring("sdo_read: slave %i, reg 0x%X, sub index %i", slave, index, sub));
    const int working_counter = ec_SDOread(slave, index, sub, FALSE, &val_size, value, EC_TIMEOUTRXM);
    if (working_counter == 0) {
        logger_->fatal(logger_->fstring(
            "sdo_read: Error occurred when reading: slave %i, reg 0x%X, sub index %i", slave, index, sub));
    }
    return working_counter;
}
} // namespace march
