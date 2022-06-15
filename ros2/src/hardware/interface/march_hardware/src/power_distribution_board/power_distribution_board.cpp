#include "march_hardware/power_distribution_board/power_distribution_board.h"
#include "march_hardware/error/hardware_exception.h"
#include "march_hardware/ethercat/pdo_types.h"

#include <ros/ros.h>

namespace march {
PowerDistributionBoard::PowerDistributionBoard(
    const Slave& slave, uint8_t byte_offset)
    : Slave(slave)
    , byte_offset_(byte_offset) {};

PowerDistributionBoardData PowerDistributionBoard::read()
{
    std::array<bit32, POWER_DISTRIBUTION_BOARD_DATA_LENGTH> data {};
    for (unsigned int i = 0; i < data.size(); i++) {
        // Increment the offset by 4 bytes each iteration
        data[i] = this->read32(byte_offset_ + i * sizeof(float));
    }
    static_assert(POWER_DISTRIBUTION_BOARD_DATA_LENGTH == 15);
    return { data[0], data[1], data[2], data[3], data[4], data[5], data[6],
        data[7], data[8], data[9], data[10], data[11], data[12], data[13],
        data[14] };
}

} // namespace march
