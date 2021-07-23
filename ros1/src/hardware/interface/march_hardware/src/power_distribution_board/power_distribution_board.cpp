#include "march_hardware/error/hardware_exception.h"
#include <march_hardware/power_distribution_board/power_distribution_board.h>

#include <ros/ros.h>

namespace march {
PowerDistributionBoard::PowerDistributionBoard(
    const Slave& slave, uint8_t byte_offset)
    : Slave(slave)
    , byte_offset_(byte_offset) {};

PowerDistributionBoardData PowerDistributionBoard::read()
{
    PowerDistributionBoardData data {};
    for (unsigned int i = 0; i < POWER_DISTRIBUTION_BOARD_DATA_LENGTH; i++) {
        data[i] = this->read32(byte_offset_ + i * sizeof(float)).f;
    }
    static_assert(POWER_DISTRIBUTION_BOARD_DATA_LENGTH == 15);
    return data;
}

} // namespace march
